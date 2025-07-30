/*
 * app_events.c
 *
 *  Created on: Jul 16, 2025
 *      Author: marcomiglio
 */

#include "app_events.h"



/*
 * This function is called to process a RX event when a complete payload
 * has been received.
 *
 * If the payload read from the RFM95 is valid, the function will process it and
 * update the RX FIFO accordingly.
 * If an error is detected in the received payload (either CRC fail or bad format pkt),
 * it will be dropped and the rfm returns in RX continuous mode for a new cycle.
 *
 * @param rfm95_handle_t* h_rfm   handler for the RFM95 LoRa transceiver
 * @param h_rx_tx*        h_fifo  rx-tx queues handler;
 * @param bc_pkt*         rx_pkt  ack pkt containing all the infos
 *
 * @return: events_flags, bit-field structure with all possible errors.
 *
 *          ERROR FLAGS (in .err_flags field):
 *          - EVT_RFM_SPI_ERR set if SPI error occured between RFM95 and MCU
 *          - EVT_RFM_RX_ERR set if error occured during RX operation (either due to SPI, or CRC failure)
 *          - EVT_RX_FIFO_FULL is set if the pkt was not added due to full RX FIFO
 *          - EVT_BAD_PKT_FORMAT is set if payload cannot be processed due to bad formatting
 *
 *          STATUS FLAGS (in .status_flags field):
 *          - EVT_SCHEDULE_TX set if a new pkt has been added to the RX FIFO. The controller should TX
 *            that pkt (correpsoning to the tail in the LL TX buffer) in the next TX event.
 *
 */
events_flags on_rx_event(rfm95_handle_t* h_rfm, h_rx_tx* h_fifo){

  /* track error flags */
  events_flags app_flags = {0};

  /* strcuture to hold all the informations */
  bc_pkt rx_pkt = {0};

  /* 1st track SNR and RSSI of the last received packet */
  int8_t  snr;
  int16_t rssi;
  if(!rfm95_getSNR(h_rfm, &snr))   app_flags.err_flags |= EVT_RFM_SPI_ERR;
  if(!rfm95_getRSSI(h_rfm, &rssi)) app_flags.err_flags |= EVT_RFM_SPI_ERR;

  /* set standby mode to read data from rfm95 */
  if(!rfm95_stdby(h_rfm)) app_flags.err_flags |= EVT_RFM_SPI_ERR;

  /* read received data */
  if(!rfm95_receive(h_rfm, &rx_pkt.pl[0], &rx_pkt.pl_len))app_flags.err_flags |= EVT_RFM_RX_ERR;

  /* set RFM95 back to continuous RX mode */
  if(!rfm95_enter_rx_mode(h_rfm)) app_flags.err_flags |= EVT_RFM_SPI_ERR;

  /* If any error occurred, stop code here */
  //if(app_flags.err_flags != 0) return app_flags;

  uint8_t sample_buffer[ENV_NODE_PYL_SIZE + 5];
  for (uint16_t i = 0; i < ENV_NODE_PYL_SIZE + 5; i++) {
    sample_buffer[i] = (uint8_t)(i & 0xFF);  // fill with some pattern
  }

  memcpy(rx_pkt.pl, sample_buffer, ENV_NODE_PYL_SIZE + 5);
  rx_pkt.pl_len = ENV_NODE_PYL_SIZE + 5;


  /* clear flags */
  app_flags.err_flags &= 0x00;

  /* Extract informations */

  rx_pkt.ack = false;                                   // init as NACK
  rx_pkt.tx_attempts = 0;                               // full attempts available
  rx_pkt.nodeID = rx_pkt.pl[NODE_ID_POS];               // byte0 = nodeID

  // byte1 and byte2 = pktID
  rx_pkt.pktID = (rx_pkt.pl[PKT_ID_MSB_POS] << 8) | rx_pkt.pl[PKT_ID_LSB_POS];

  // rx_bcID and tx_bcID depend on subsequent conditions

  if (rx_pkt.pl_len == ENV_NODE_PYL_SIZE) {       /* receiving from an ENV NODE */

    app_flags = process_envNode_up(h_fifo, &rx_pkt, rssi);

  } else if (rx_pkt.pl_len > ENV_NODE_PYL_SIZE) { /* receiving fron BC NODE -> some hops happened */

    /*
     * if receiving from bcNode, at leat one Hop happened
     * -> extarct 1st bc ID and last bc ID in the hopping sequence
     */
    rx_pkt.tx_bcID = rx_pkt.pl[rx_pkt.pl_len - 1];  // last RX byte corresponds to the BC_ID of the last BC in the hop-sequence

    // get the BC_ID of the 1st BC in the hop-sequence
    rx_pkt.rx_bcID = rx_pkt.pl[BC_ID1_POS];

    if (rx_pkt.tx_bcID > MY_BC_ID) {    /* RX from node further in the BCs sequence --> UPLINK */

      app_flags = process_bcNode_up(h_fifo, &rx_pkt);

    } else {                            /* RX from node ahead in the BCs sequence --> ACK */

      app_flags = process_bcNode_ack(h_fifo, &rx_pkt);

    }

  } else {                              /* BAD PKT format */

    app_flags.err_flags |= EVT_BAD_PKT_FORMAT;

  }

  return app_flags;
}

/*
 * This function is called to process a TX event when a timer IRQ is triggered.
 *
 * The TX queue is handled through a linked list. When new PKTs are 1st received the TX queue
 * is accessed from the tail to complete the 1st TX in the shortest possible time. When no "new PKTs"
 * are present (i.e. all the PKTs in the TX BUFF have at least 1 TX attempt), TX events are SHCEDULED with
 * longer wait time for retransmissions.
 *
 * The RFM95 is automatically put into TX mode, and in the end reverts to continuous TX mode if no erorrs
 * were detected.
 *
 * To schedule new TX events the user should process the .status_flags returned in the events_flags structure.
 *
 * @param rfm95_handle_t* h_rfm   handler for the RFM95 LoRa transceiver
 * @param h_rx_tx*        h_fifo  rx-tx queues handler
 *
 * @return: events_flags, bit-field structure with all possible errors.
 *
 *          ERROR FLAGS (in .err_flags field):
 *          - EVT_RFM_SPI_ERR set if SPI error occured between RFM95 and MCU
 *
 *          STATUS FLAGS (in .status_flags field):
 *          - EVT_RFM_MODEM_RX is set if the modem is locked into a preable/payload when entering this function
 *                             --> The TX event should be rescheduled with short wait
 *          - EVT_TX_FIFO_EMPTY is set if no pkts are waiting for TX (all TX more than TX_ATTEMPTS_THRESHOLD times)
 *                             --> TX timer shouldn't be reactivated
 *          - EVT_SCHEDULE_PRI_TX is set if the TX queue contains other "new PKTs" waiting for 1st TX
 *                             --> The next TX event should be rescheduled with short wait time
 *          - EVT_SCHEDULE_TX set if the TX queue contains only "old PKTs" (i.e. waiting for reTX)
 *                             --> The next TX event should be rescheduled with long wait time
 *
 */
events_flags on_tx_event(rfm95_handle_t* h_rfm, h_rx_tx* h_fifo){
  /* track error flags */
  events_flags app_flags = {0};
  uint16_t fifo_status;
  uint16_t tx_idx = 0;
  uint8_t rfm_reg;

  // check RFM95 MODEM status:
  if(!rfm95_getModemStatus(h_rfm, &rfm_reg)) app_flags.err_flags |= EVT_RFM_SPI_ERR;

  /* If any error occurred, stop code here */
  if(app_flags.err_flags != 0) return app_flags;

  if((rfm_reg & 0x03) != 0) {
    /*
     * RFM modem in "signal deteted" or "signal synchronized
     * -> Skip this TX and wait for the end of the event
     */
    app_flags.status_flags |= EVT_RFM_MODEM_RX;
    return app_flags;
  }

  /* clear flags */
  app_flags.err_flags &= 0x00;

  uint8_t pyl_buff[LORA_PAYLOAD_MAX_SIZE];
  uint8_t pyl_len;

  /* 1st try looking for "new PKTs" with no TX yet -> Higher priority */
  tx_idx = get_nextTX_pkt(h_fifo, TX_SEQ_ENTRY_TAIL, pyl_buff, &pyl_len);

  if(tx_idx == LL_BUFF_EMPTY) {                   /* No events to TX */

    app_flags.status_flags |= EVT_TX_FIFO_EMPTY;
    return app_flags;

  } else if(tx_idx == RX_BUFF_IDX_NOT_DEFINED) {  /* No "New PKTs present */

    /* TX oldest one */
    tx_idx = get_nextTX_pkt(h_fifo, TX_SEQ_ENTRY_HEAD, pyl_buff, &pyl_len);

  } else {
    // RFU...
  }

  /* TX payload here */
  if (!rfm95_send(h_rfm, pyl_buff, pyl_len)) app_flags.err_flags |= EVT_RFM_SPI_ERR;
  if(app_flags.err_flags != 0) return app_flags;

  /* Set RFM back to RX mode */
  if (!rfm95_enter_rx_mode(h_rfm)) app_flags.err_flags |= EVT_RFM_SPI_ERR;
  if(app_flags.err_flags != 0) return app_flags;

  /* Update TX attempts for this PKT */
  h_fifo->h_rx[tx_idx].pkt.tx_attempts += 1;
  if (h_fifo->h_rx[tx_idx].pkt.tx_attempts >= BC_TX_ATTEMPTS) { /* remove this PKT from TX queue */

    tx_queue_remove(h_fifo, tx_idx);

  }

  /* get updated TX queue status */
  fifo_status = get_nextTX_pri(h_fifo);

  if (fifo_status == LL_BUFF_EMPTY){                /* TX BUFF EMPTY -> Sop TX timer */

    app_flags.status_flags |= EVT_TX_FIFO_EMPTY;

  } else if (fifo_status == TX_BUFF_PRI) {          /* TX Buff has "new PKTs" -> short wait */

    app_flags.status_flags |= EVT_SCHEDULE_PRI_TX;

  } else if (fifo_status == TX_BUFF_NO_PRI) {       /* TX Buff has "old PKTs" -> longer wait for reTX*/

    app_flags.status_flags |= EVT_SCHEDULE_TX;

  } else {                                          /* RFU... */
    //RFU...
  }

  return app_flags;
}

/*
 * This function is called to process an UPLINK payload (i.e.
 * travelling from a node behind BC_ID < MY_BC_ID). The entire RX-buff
 * and the following decisions are taken:
 *
 * given the received pkt (nodeID, pktID, 1st bcID)
 *
 * - If different node IDs are present in the FIFO -> rx pkt is added
 *
 * - If coincident node IDs are found:
 *   - drop all pkts with pkt ID smaller than the rx pkt ID (older versions are removed)
 *   - for coincident pkt IDs and coincident 1st bc IDs:
 *        # if ACK or transmitted too many times, ignore the packet
 *        # otherwise keep only one copy (the one with less hops - less bytes to TX)
 *
 * - for coincident pkt IDs but differet 1st bc IDs, treat as different pkt since they
 *   carry different information
 *
 * - if rx pkt ID < my pkt ID, ignore since it's an old information
 *
 * In the end, if that pkt doesn't match any in the RX fifo, add it.
 *
 *
 * @param h_rx_tx*   h_fifo  rx-tx queues handler;
 * @param bc_pkt*    rx_pkt  ack pkt containing all the infos
 *
 * @return: events_flags, bit-field structure with all possible errors:
 *
 *          - the bit EVT_RX_FIFO_FULL in the .err_flags field is set if the pkt was not added due to an error
 *
 *          - the bit EVT_SCHEDULE_TX in the .status_flags field is set if a new pkt was added. The controller
 *            should TX that pkt (tail in the LL sequence) in the next TX event.
 *
 */
events_flags process_bcNode_up(h_rx_tx* h_fifo, bc_pkt* rx_pkt){
  /* track error flags */
  events_flags app_flags = {0};
  uint16_t fifo_err_status;

  bool add_new_pkt = true;           // track wether this pkt has to be added
  bool tx_new_pkt = true;            // new pkts TX immediatey after RX
  uint16_t add_idx = BUFF_FIFO_SIZE; // track insert idx (no additional computational cost)

  /* Scan through the RX buffer */
  for (uint16_t i = 0; i < BUFF_FIFO_SIZE; i++){

    rnode c_node = h_fifo->h_rx[i];

    if (c_node.slot_free == true){

      // Empty slot -> eligible for insertion
      if (i <= add_idx) add_idx = i;
      continue;

    } else { /* This slot in the FIFO contains a valid PKT */

      if (rx_pkt->nodeID != c_node.pkt.nodeID) { /* The RX pkt has a different Node ID */

        // Skip... Move to the next valid pkt in the RX buff
        continue;

      } else { /* Coincident Node IDs */

        if ((rx_pkt->pktID > c_node.pkt.pktID)) {       /* Newer packet received --> replace older pkts (older pkt IDs) */

          /*
           * remove old pkt from queue
           * -> in the end replaced by received one
           */
          fifo_err_status = remove_pkt(h_fifo, i);

          // Empty slot -> eligible for insertion
          if (i <= add_idx) add_idx = i;

        } else if((rx_pkt->pktID == c_node.pkt.pktID)) {/* Same identical pktID (i.e. same information) */

          if (rx_pkt->rx_bcID == c_node.pkt.rx_bcID) {  /* Same "receive point" */
            // same 1st bcID in the sequence of hops

            if (c_node.pkt.ack || c_node.pkt.tx_attempts >= BC_TX_ATTEMPTS) { /* ACK or TX too many times */

              /*
               * In the FIFO there's already a pkt identical to the received one.
               * But it has been ACK or TX many times -> ignore received packet
               * and do not add to the buffer.
               */
              add_new_pkt = false;

            } else { /* Not ACK and some TX attempts left */

              /*
               * In my FIFO i have a pkt identical to the RX one, but
               * they followed different hops -> I keep only the shortest one
               * (less bytes to TX)
               */
              if (c_node.pkt.pl_len <= rx_pkt->pl_len) {

                // FIFO contains a pkt shorter than the received one
                add_new_pkt = false;

              } else {

                // this pkt is not "new"
                tx_new_pkt = false;

                // received pkt is shorter -> replace (maintain tx attempts)
                rx_pkt->tx_attempts = c_node.pkt.tx_attempts;

                fifo_err_status = remove_pkt(h_fifo, i);

                // Empty slot -> eligible for insertion
                if (i <= add_idx) add_idx = i;

              }

            }

            /*
             * pkt with identical node IDs, pkt IDs, 1st rx point.
             * Only one copy can be present in the RX fifo, so i can block
             * the RX scan procedure.
             */
            break;

          } else { /*  Same pkt IDs, but different 1st bc ID*/

            /*
             *  The packet was received in different points along the sequence of hops
             *  Treat it as a new packet...
             */

            // skip...
            continue;

          }

        } else {                                        /* Same node ID but older pkt ID -> ignore */

          /*
           * - rx_pktID older than mine (in my RX FIFO i have a newwer pkt
           *   from the same environmental node) -> do not propagate
           */
          add_new_pkt = false;

          /*
           * pkt with identical node IDs, but in my RX fifo
           * i already have "newer" informations from the same ENV node (i.e.
           * pktIDs older than RX pktID).
           * All the other pkts from the same node ID will have older
           * pkt IDs than the received one!
           */
          break;

        }

      }

    }

  } /* RX FIFO scan completed */

  if (add_new_pkt == true) { /* If the flag is still set -> Add pkt */

    // add my bc ID informations:
    rx_pkt->pl[rx_pkt->pl_len] = MY_BC_ID;
    rx_pkt->pl_len = rx_pkt->pl_len + 1;

    fifo_err_status = add_pkt(h_fifo, add_idx, rx_pkt);
    app_flags.err_flags |= (fifo_err_status == RX_BUFF_FULL) ? EVT_RX_FIFO_FULL : 0;

    if ((app_flags.err_flags == 0) && tx_new_pkt) {
      // If adding a new pkt flag a TX event -> Try to travel through the BCs as quick as possible
      app_flags.status_flags |= EVT_SCHEDULE_TX;

    }

  }

  return app_flags;
}

/*
 * This function is called to process an UPLINK payload from an ENV node.
 * The entire RX-buff is scanned and the following decisions are taken:
 *
 * given the received pkt (nodeID, pktID, 1st bcID)
 *
 * - If different node IDs are present in the FIFO -> rx pkt is added
 *   (pkt never seen before)
 *
 * - If coincident node IDs are found:
 *   - drop all pkts with pkt ID smaller than the rx pkt ID (older versions are removed)
 *   - for coincident pkt IDs ignore the received pkt (my RX FIFO has already seen
 *     thet pkt at least once)
 * - if rx pkt ID < my pkt ID, ignore since it's an old information
 *
 * In the end, if that pkt doesn't match any in the RX fifo, it adds it and schedule
 * a TX event.
 *
 * OBS: the ENV nodes can take the following decisions:
 *      - add redundancy by keeping identical pktID over subsequent TX attempts. In this
 *        case the purpose is simply to ensure that the pkt reaches the BCs sequence.
 *
 *      - add redundancy by increasing pktID over subsequent TX attempts. In this case the
 *        redundant pkt is treated as a new information -> Even if a BC has already seen
 *        that pkt it will treat as a new one and trigger a new TX sequence through the BC
 *        sequence.
 *
 * @param h_rx_tx*   h_fifo  rx-tx queues handler;
 * @param bc_pkt*    rx_pkt  ack pkt containing all the infos
 * @param int16_t    rssi    RSSI registered by this BC node on RX
 *
 * @return: events_flags, bit-field structure with all possible errors:
 *
 *          - the bit EVT_RX_FIFO_FULL in the .err_flags field is set if the pkt was not added due to an error
 *
 *          - the bit EVT_SCHEDULE_TX in the .status_flags field is set if a new pkt was added. The controller
 *            should TX that pkt (tail in the LL sequence) in the next TX event.
 *
 */
events_flags process_envNode_up(h_rx_tx* h_fifo, bc_pkt* rx_pkt, int16_t rssi){
  /* track error flags */
  events_flags app_flags = {0};
  uint16_t fifo_err_status;

  bool add_new_pkt = true;           // track wether this pkt has to be added
  uint16_t add_idx = BUFF_FIFO_SIZE; // track insert idx (no additional computational cost)

  for(uint16_t i = 0; i < BUFF_FIFO_SIZE; i++) {

    rnode c_node = h_fifo->h_rx[i];

    if (c_node.slot_free == true){

      // Empty slot -> eligible for insertion
      if (i <= add_idx) add_idx = i;
      continue;

    } else { /* This slot in the FIFO contains a valid PKT */

      if (rx_pkt->nodeID != c_node.pkt.nodeID) { /* The RX pkt has a different Node ID */

        // Skip... Move to the next valid pkt in the RX buff
        continue;

      } else { /* Coincident Node IDs */

        if ((rx_pkt->pktID > c_node.pkt.pktID)) {       /* Newer packet received --> replace older pkts (older pkt IDs) */

          /*
           * remove old pkt from queue
           * -> in the end replaced by received one
           */
          fifo_err_status = remove_pkt(h_fifo, i);

          // Empty slot -> eligible for insertion
          if (i <= add_idx) add_idx = i;

        } else if((rx_pkt->pktID == c_node.pkt.pktID)) {/* Same identical pktID (i.e. same information) */

          /*
           * The env node is adding redundancy, but i have already seen
           * this PKT -> ignore
           */
          add_new_pkt = false;

        } else {                                        /* Same node ID but older pkt ID -> ignore */

          /*
           * - rx_pktID older than mine (in my RX FIFO i have a newer pkt
           *   from the same environmental node) -> do not propagate
           */
          add_new_pkt = false;

          /*
           * pkt with identical node IDs, but in my RX fifo
           * i already have "newer" informations from the same ENV node (i.e.
           * pktIDs older than RX pktID).
           * All the other pkts from the same node ID will have older
           * pkt IDs than the received one!
           */
          break;

        }

      }

    }

  } /* RX FIFO scan completed */

  if (add_new_pkt == true) { /* If the flag is still set -> Add pkt */

    // Receiving from ENV node -> add RSSI
    rx_pkt->pl[rx_pkt->pl_len]   = (uint8_t)((rssi >> 8) & 0xFF);
    rx_pkt->pl[rx_pkt->pl_len+1] = (uint8_t)(rssi & 0xFF);
    rx_pkt->pl_len += 2;

    // add my bc ID informations:
    rx_pkt->pl[rx_pkt->pl_len] = MY_BC_ID;
    rx_pkt->pl_len = rx_pkt->pl_len + 1;

    fifo_err_status = add_pkt(h_fifo, add_idx, rx_pkt);
    app_flags.err_flags |= (fifo_err_status == RX_BUFF_FULL) ? EVT_RX_FIFO_FULL : 0;

    if (app_flags.err_flags == 0) { // If no errors occured -> Schedule TX event

      app_flags.status_flags |= EVT_SCHEDULE_TX;

    }

  }

  return app_flags;
}

/*
 * This function is called to process an ACK payload (i.e.
 * travelling from a node ahead with BC_ID < MY_BC_ID). The entire RX-buff
 * is processed to check the ACK.
 *
 * ACK is considered for all the packets in the FIFO that:
 * - have the same node ID
 * - have smaller pktID (i.e. a node ahead has "newer pkt", so
 *   stop transmitting older ones)
 * - have same pktID and same 1st bc ID (i.e. the same identical
 *   information)
 *
 * @param h_rx_tx*   h_fifo  rx-tx queues handler;
 * @param bc_pkt*    rx_pkt  ack pkt containing all the infos
 *
 * @return: events_err_flags, bit-field structure with all possible errors.
 *                   --> no real errors happen here (only flags used for debug).
 *
 */
events_flags process_bcNode_ack(h_rx_tx* h_fifo, bc_pkt* rx_pkt){
  /* track error flags */
  events_flags app_flags = {0};
  uint16_t fifo_err_status;

  /* Scan through the RX buffer */
  for (uint16_t i = 0; i < BUFF_FIFO_SIZE; i++){

    rnode c_node = h_fifo->h_rx[i];

    if (c_node.slot_free == true){

      // Empty slot -> skip... (this happens to perform removal in O(1))
      continue;

    } else { /* This slot in the FIFO contains a valid PKT */

      if (rx_pkt->nodeID != c_node.pkt.nodeID) { /* The ACK pkt has a different Node ID */

        // Skip... Move to the next valid pkt in the RX buff
        continue;

      } else { /* Coincident Node IDs */

        if ((rx_pkt->pktID > c_node.pkt.pktID) ||        /* Newer or same identical packet detected --> Set ACK */
            ((rx_pkt->pktID == c_node.pkt.pktID) && (rx_pkt->rx_bcID == c_node.pkt.rx_bcID))) {

          /* Node ahead has newer pktID, or same identical pkt (ACK received) */
          h_fifo->h_rx->pkt.ack = true;

          /* remove from tx queue */
          fifo_err_status = tx_queue_remove(h_fifo, i);

        } else {                                        /* Same node ID but different infos --> no ACK*/

          /*
           * - rx_pktID older than mine -> do not ACK
           * - same pkt ID but differest 1st bcID in the hop sequence (i.e. different rx point)
           */

          // skip...
          continue;

        }

      }

    }

  }

  return app_flags;
}
