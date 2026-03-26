/*
 * app_events.c
 *
 *  Created on: Jul 16, 2025
 *      Author: marcomiglio
 */

#include "app_events.h"


// ----------------------------------------------- STATIC FUNCTIONS --------------------------------------------------------

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
 *        # keep only the copy in my RX FIFO
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
static events_flags process_bcNode_up(h_rx_tx* h_fifo, bc_pkt* rx_pkt){
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
            /*
             * In my FIFO i have a pkt identical to the RX one, but
             * they followed different hops
             */

            add_new_pkt = false;

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
 * This function is called to process an UPLINK ALARM payload (i.e.
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
 *        # keep only the copy in my RX FIFO (i.e. no new PKT is added)
 *        # If the RX PKT has "ACK bit" set in the MASK field, ignore it since it represents a broadcast
 *          trasnsmission intended as an ACK for a node behind me.
 *        # If the copy in my RX FIFO was already ACK -> schedule an ACK message for the RX pkt
 *          (A node behind me hasn't receive ACK for its ALARM!)
 *        # If the copy in my RX FIFO has not been ACK -> nothing to do (A node behind me is requiring ACK,
 *          but my copy is still present in the TX FIFO, sooner or later it wil be ReTX)
 *
 * - for coincident pkt IDs but differet 1st bc IDs, treat as different pkt since they
 *   carry different information (different receive point)
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
 *          - the bit EVT_SCHEDULE_ACK in the .status_flags field is set if an ACK pkt was added. The controller
 *            should TX that pkt (tail in the LL sequence) in the next TX event.
 *
 *          - the bit EVT_SCHEDULE_TX in the .status_flags field is set if a new pkt was added. The controller
 *            should TX that pkt (tail in the LL sequence) in the next TX event.
 *
 */
static events_flags process_bcNode_up_alarm(h_rx_tx* h_fifo, bc_pkt* rx_pkt){
  /* track error flags */
  events_flags app_flags = {0};
  uint16_t fifo_err_status;

  bool alarm_ack_pkt = false;        // track if the new PKT is used to ACK an ALARM message
  uint16_t rx_buff_idx;              // RX buff IDX used to track which PKT is requiring an ACK

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
            /*
             * In my FIFO i have a pkt identical to the RX one, but
             * they followed different hops
             *
             * In alarm mode -> a node behind hasn't receive an ACK yet
             */

            add_new_pkt = false; // don't need to add in RX FIFO

            bool is_ack = ((rx_pkt->pl[MASK_POS] & MASK_ALARM_ACK) >> ACK_BIT_POS) & 0x01;
            if (is_ack) {
              /*
               * This uplink is a broadcast TX used as an ACK for a PKT
               * behind my BC node position -> ignore...
               */
              break;
            }

            if (c_node.pkt.ack){
              /*
               *  My packet was already ACK -> schedule a single TX to send an ACK
               */
              rx_buff_idx = i;

              // init to MAX limit -> after one TX it is removed from TX queue
              h_fifo->h_rx[i].pkt.tx_attempts = ALRM_PKT_SINGLE_TX;

              // set ACK bit in the MASK
              h_fifo->h_rx[i].pkt.pl[MASK_POS] |= MASK_ALARM_ACK;

              // flag single TX event
              alarm_ack_pkt = true;

            } else {
              // Also my PKT hasn't been ACK -> already present in TX queue
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

        } else {                                        /* Same node ID but older pkt ID */

          /*
           * - rx_pktID older than mine (in my RX FIFO i have a newwer pkt
           *   from the same environmental node) -> do not propagate
           */
          add_new_pkt = false;

          bool is_ack = ((rx_pkt->pl[MASK_POS] & MASK_ALARM_ACK) >> ACK_BIT_POS) & 0x01;
          if (is_ack) {
            /*
             * This uplink is a broadcast TX used as an ACK for a PKT
             * behind my BC node position -> ignore...
             */
            break;
          }

          if (c_node.pkt.ack){
            /*
             *  My packet was already ACK -> schedule a single TX to send an ACK
             */
            rx_buff_idx = i;

            // init to MAX limit -> after one TX it is removed from TX queue
            h_fifo->h_rx[i].pkt.tx_attempts = ALRM_PKT_SINGLE_TX;

            // set ACK bit in the MASK
            h_fifo->h_rx[i].pkt.pl[MASK_POS] |= MASK_ALARM_ACK;

            // flag single TX event
            alarm_ack_pkt = true;

          } else {
            // Also my PKT hasn't been ACK -> already present in TX queue
          }

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

  if (alarm_ack_pkt == true) {      /* Add PKT only in the TX buffer (already present in RX FIFO) */

    fifo_err_status = tx_queue_add(h_fifo, rx_buff_idx);

    app_flags.err_flags |= (fifo_err_status == RX_BUFF_FULL) ? EVT_RX_FIFO_FULL : 0;

    if (app_flags.err_flags == 0) {
      // If adding an ACK pkt -> flag event
      app_flags.status_flags |= EVT_SCHEDULE_ACK;
    }

  } else if (add_new_pkt == true) { /* If the flag is still set -> Add pkt */

    // add my bc ID informations:
    rx_pkt->pl[rx_pkt->pl_len] = MY_BC_ID;
    rx_pkt->pl_len = rx_pkt->pl_len + 1;

    // make sure ACK bit not set
    rx_pkt->pl[MASK_POS] &= ~MASK_ALARM_ACK;

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
static events_flags process_envNode_up(h_rx_tx* h_fifo, bc_pkt* rx_pkt, int16_t rssi){
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

          if (c_node.pkt.rx_bcID == MY_BC_ID){
            /*
             * The env node is adding redundancy, but i have already seen
             * this PKT -> ignore
             */
            add_new_pkt = false;
            break;

          } else {

            /*
             * I have a PKT form the same ENV NODE, with same PKTid,
             * but the receive points are different -> treat as different infos
             */
            continue;

          }


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

    // modify payload (change ENV_NODE_SYNC_WORD with BC_NODE_SYNC_WORD)
    rx_pkt->pl[SYNC_WORD_POS]   = (uint8_t)((SYNC_WORD_BC >> 8) & 0xFF);
    rx_pkt->pl[SYNC_WORD_POS+1] = (uint8_t)( SYNC_WORD_BC & 0xFF);

    // Receiving from ENV node -> add RSSI
    rx_pkt->pl[rx_pkt->pl_len]   = (uint8_t)((rssi >> 8) & 0xFF);
    rx_pkt->pl[rx_pkt->pl_len+1] = (uint8_t)(rssi & 0xFF);
    rx_pkt->pl_len += 2;

    // add my bc ID informations:
    rx_pkt->rx_bcID = MY_BC_ID;
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
static events_flags process_bcNode_ack(h_rx_tx* h_fifo, bc_pkt* rx_pkt){
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
          h_fifo->h_rx[i].pkt.ack = true;

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

/*
 * Handle PKT TX event for "normal operation" (no ALARM).
 *
 * @param rfm95_handle_t* h_rfm   handler for the RFM95 LoRa transceiver
 * @param h_rx_tx*        h_fifo  rx-tx queues handler;
 *
 * @return: events_flags, bit-field structure with all possible errors.
 *
 *          ERROR FLAGS (in .err_flags field):
 *          - EVT_RFM_SPI_ERR set if SPI error occured between RFM95 and MCU
 *
 *          STATUS FLAGS (in .status_flags field):
 *          - EVT_TX_FIFO_EMPTY is set if no pkts are waiting for TX (all TX more than TX_ATTEMPTS_THRESHOLD times)
 *                             --> TX timer shouldn't be reactivated
 *          - EVT_SCHEDULE_PRI_TX is set if the TX queue contains other "new PKTs" waiting for 1st TX
 *                             --> The next TX event should be rescheduled with short wait time
 *          - EVT_SCHEDULE_TX set if the TX queue contains only "old PKTs" (i.e. waiting for reTX)
 *                             --> The next TX event should be rescheduled with long wait time
 */
static events_flags handle_PktTX(rfm95_handle_t* h_rfm, h_rx_tx* h_fifo){
  /* track error flags */
  events_flags app_flags = {0};
  uint16_t fifo_status;
  uint16_t tx_idx = 0;

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
 * Handle PKT TX event for ALARM events.
 *
 * @param rfm95_handle_t* h_rfm   handler for the RFM95 LoRa transceiver
 * @param h_rx_tx*        h_fifo  rx-tx queues handler;
 *
 * @return: events_flags, bit-field structure with all possible errors.
 *
 *          ERROR FLAGS (in .err_flags field):
 *          - EVT_RFM_SPI_ERR set if SPI error occured between RFM95 and MCU
 *
 *          STATUS FLAGS (in .status_flags field):
 *          - EVT_TX_FIFO_EMPTY is set if no pkts are waiting for TX (all TX more than TX_ATTEMPTS_THRESHOLD times)
 *                             --> controller should schedule a timer to quit ALARM mode.
 *          - EVT_SCHEDULE_PRI_TX is set if the TX queue contains "new PKTs" waiting for 1st TX or ACK PKTs
 *                             --> The next TX event should be rescheduled with short wait time
 *          - EVT_SCHEDULE_TX set if the TX queue contains only "old PKTs" (i.e. waiting for reTX)
 *                             --> The next TX event should be rescheduled with longer wait time
 */
static events_flags handle_AlarmTX(rfm95_handle_t* h_rfm, h_rx_tx* h_fifo){
  /* track error flags */
  events_flags app_flags = {0};
  uint16_t fifo_status;
  uint16_t tx_idx = 0;

  uint8_t pyl_buff[LORA_PAYLOAD_MAX_SIZE];
  uint8_t pyl_len;

  /* 1st try looking for "new PKTs" with no TX yet -> Higher priority */
  tx_idx = get_nextTX_pkt_alrm(h_fifo, TX_SEQ_ENTRY_TAIL, pyl_buff, &pyl_len);

  if(tx_idx == LL_BUFF_EMPTY) {                   /* No events to TX */

    app_flags.status_flags |= EVT_TX_FIFO_EMPTY;
    return app_flags;

  } else if(tx_idx == RX_BUFF_IDX_NOT_DEFINED) {  /* No "New PKTs present */

    /* Follow head -> tail order (all PKTs equally needs an ACK) */
    tx_idx = get_nextTX_pkt_alrm(h_fifo, TX_SEQ_SCAN, pyl_buff, &pyl_len);

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
  if (h_fifo->h_rx[tx_idx].pkt.tx_attempts >= ALARM_MAX_TX_ATTEMPTS) { /* remove this PKT from TX queue */

    tx_queue_remove(h_fifo, tx_idx);

  }

  /* get updated TX queue status */
  fifo_status = get_nextAlrmTX_pri(h_fifo);

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


// ---------------------------------------------------- END OF STATIC BLOCK --------------------------------------------------------------


// ------------------------------------------------------ PUBLIC FUNCTIONS ---------------------------------------------------------------

/*
 * This function is called to process a RX event when a complete payload
 * has been received.
 *
 * If the payload read from the RFM95 is valid, the function will process it and
 * update the RX FIFO accordingly.
 * If an error is detected in the received payload (either CRC fail or bad format pkt),
 * it will be dropped and the rfm returns in RX continuous mode for a new cycle.
 *
 * If ALARM bit is detected in the MASK field, the handler automatically clears RX and TX FIFOs and
 * prepares it for handling a sequence of ALARM events. In ALARM mode, normal pkts are dropped until the
 * system reverts to standard operation.
 *
 * @param rfm95_handle_t* h_rfm   handler for the RFM95 LoRa transceiver
 * @param h_rx_tx*        h_fifo  rx-tx queues handler
 *
 * @return: events_flags, bit-field structure with all possible errors.
 *
 *          ERROR FLAGS (in .err_flags field):
 *          - EVT_RFM_SPI_ERR set if SPI error occured between RFM95 and MCU
 *          - EVT_RFM_RX_ERR set if error occured during RX operation (either due to SPI, or CRC failure)
 *          - EVT_RX_FIFO_FULL is set if the pkt was not added due to full RX FIFO
 *          - EVT_BAD_PKT_FORMAT is set if payload cannot be processed due to bad formatting or
 *                               neither SYNC_WORD_ENV nor SYNC_WORD_BC were recognized
 *
 *          STATUS FLAGS (in .status_flags field):
 *          - EVT_SCHEDULE_TX set if a new pkt has been added to the RX FIFO. The controller should TX
 *            that pkt (correpsoning to the tail in the LL TX buffer) in the next TX event.
 *          - EVT_ALARM_PKT is set if ALARM bit in the MASK field of the received PKT was set.
 *          - EVT_SCHEDULE_ACK is set during ALARM mode if an ACK pkt was added. The controller
 *            should TX that pkt (tail in the LL sequence) in the next TX event (i.e. treat as high priority).
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
  if(app_flags.err_flags != 0) {
    return app_flags;
  }

  /* read received sync word */
  uint16_t rx_sync = (uint16_t)((rx_pkt.pl[SYNC_WORD_POS] << 8) | rx_pkt.pl[SYNC_WORD_POS+1]);

  /* preliminary check on payload size and SYNC words */
  if (!( ((rx_pkt.pl_len == ENV_NODE_PYL_SIZE) && (rx_sync == SYNC_WORD_ENV)) ||
         ((rx_pkt.pl_len >= BC_NODE_MIN_PYL_SIZE) && (rx_sync == SYNC_WORD_BC)) )) {

    app_flags.err_flags |= EVT_BAD_PKT_FORMAT;
    return app_flags;

  }

//  /* clear flags */
//  app_flags.err_flags &= 0x00;

  /* Extract informations */
  rx_pkt.ack = false;                                   // init as NACK
  rx_pkt.tx_attempts = 0;                               // full attempts available
  rx_pkt.nodeID = rx_pkt.pl[NODE_ID_POS];               // extract nodeID

  // byte1 and byte2 = pktID
  rx_pkt.pktID = (rx_pkt.pl[PKT_ID_MSB_POS] << 8) | rx_pkt.pl[PKT_ID_LSB_POS];

  // get entire mask field
  uint8_t pkt_mask = rx_pkt.pl[MASK_POS];

  // get alarm bit
  bool alarm_bit_set = (pkt_mask & MASK_ALARM_BIT) >> ALARM_BIT_POS;
  if (alarm_bit_set) { /* ALARM bit SET */

    app_flags.status_flags |= EVT_ALARM_PKT;

    if (h_fifo->state == FIFO_READY) {
      /* 1st time entering ALARM state -> clear RX-TX buffs */
      init_buffers(h_fifo);
    }
  } else {             /* Normal PKT -> Handle iff FIFO is not in ALARM mode */
    if (h_fifo->state != FIFO_READY) return app_flags;
  }

  // rx_bcID and tx_bcID depend on subsequent conditions
  if ((rx_pkt.pl_len == ENV_NODE_PYL_SIZE) && (rx_sync == SYNC_WORD_ENV)) {       /* receiving from an ENV NODE */

    events_flags new_flags = process_envNode_up(h_fifo, &rx_pkt, rssi);
    app_flags.err_flags    |= new_flags.err_flags;
    app_flags.status_flags |= new_flags.status_flags;

  } else if ((rx_pkt.pl_len > ENV_NODE_PYL_SIZE) && (rx_sync == SYNC_WORD_BC)) {  /* receiving fron BC NODE -> some hops happened */

    /*
     * if receiving from bcNode, at leat one Hop happened
     * -> extarct 1st bc ID and last bc ID in the hopping sequence
     */
    rx_pkt.tx_bcID = rx_pkt.pl[rx_pkt.pl_len - 1];  // last RX byte corresponds to the BC_ID of the last BC in the hop-sequence

    // get the BC_ID of the 1st BC in the hop-sequence
    rx_pkt.rx_bcID = rx_pkt.pl[BC_ID1_POS];

    if (rx_pkt.tx_bcID > MY_BC_ID) {    /* RX from node further in the BCs sequence --> UPLINK */

      events_flags new_flags = {0};

      if (alarm_bit_set){ /* Handle Uplink for ALARM PKTs */

        new_flags = process_bcNode_up_alarm(h_fifo, &rx_pkt);

      } else {            /* Handle Uplink for normal PKTs */

        new_flags = process_bcNode_up(h_fifo, &rx_pkt);

      }

      app_flags.err_flags    |= new_flags.err_flags;
      app_flags.status_flags |= new_flags.status_flags;

    } else {                            /* RX from node ahead in the BCs sequence --> ACK */

      events_flags new_flags = process_bcNode_ack(h_fifo, &rx_pkt);
      app_flags.err_flags    |= new_flags.err_flags;
      app_flags.status_flags |= new_flags.status_flags;

    }

  } else {                              /* BAD PKT format */

    app_flags.err_flags |= EVT_BAD_PKT_FORMAT;

  }

  return app_flags;
}

/*
 * This function is called to hanlde TX events when a timer IRQ is triggered.
 *
 * The TX queue is handled through a linked list.
 *
 * IN NORMAL MODE, the priority order is the following:
 * - The TX Buffer is first scan from tail looking for high priority PKTs (0 TX attempts).
 *   If present, controller should plan a TX event with short wait time.
 * - If no high priority PKTs are present (all the PKTs have been transmitted at least once),
 *   then a FIFO order  is used to schedule retransmission events. Controller should plan ReTX of
 *   PKTs with longer wait time.
 *
 * IN ALARM MODE, the following priority order is followed:
 * - The TX Buffer is first scan from tail looking for high priority PKTs (0 TX attempts,
 *   or PKTs scheduled as an ACK response). If present, controller should plan a TX event with short wait time.
 * - If no high priority PKTs are present (no ACK scheduled, and all the PKTs have been transmitted at least once),
 *   then a sequential scan from head->tail is used to schedule retransmission events until all ALARM PKTs are ACK.
 *   Controller should plan ReTX of ALARM PKTs with longer wait time.
 *
 * The RFM95 is automatically put into TX mode, and in the end reverts to continuous RX mode if no erorrs
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
  events_flags new_flags = {0};
  uint8_t rfm_reg;

  // check RFM95 MODEM status:
  if(!rfm95_getModemStatus(h_rfm, &rfm_reg)) app_flags.err_flags |= EVT_RFM_SPI_ERR;
  if(app_flags.err_flags != 0) goto cleanup;

  if((rfm_reg & 0x03) != 0) {
    /*
     * RFM modem in "signal deteted" or "signal synchronized
     * -> Skip this TX and wait for the end of the event
     */
    app_flags.status_flags |= EVT_RFM_MODEM_RX;
    goto cleanup;
  }

  /* set standby mode to read data from rfm95 */
  if(!rfm95_stdby(h_rfm)) app_flags.err_flags |= EVT_RFM_SPI_ERR;
  if(app_flags.err_flags != 0) goto cleanup;

  if (h_fifo->state == FIFO_READY) {    /* Handle TX for normal PKTs */
    new_flags = handle_PktTX(h_rfm, h_fifo);
    app_flags.err_flags    |= new_flags.err_flags;
    app_flags.status_flags |= new_flags.status_flags;
  } else {                              /* Handle TX for ALARM PKTs */
    new_flags = handle_AlarmTX(h_rfm, h_fifo);
    app_flags.err_flags    |= new_flags.err_flags;
    app_flags.status_flags |= new_flags.status_flags;
  }

  // Return statement -> Flags + RFM in RX mode
  cleanup:
    if ((app_flags.err_flags == 0) && (h_rfm->rfm_status != RXCONTIN_MODE)){ // In case RFM was not set in RX mode

      if (!rfm95_enter_rx_mode(h_rfm)) app_flags.err_flags |= EVT_RFM_SPI_ERR;

    }
    return app_flags;
}
