/*
 * buff_manager.c
 *
 *  Created on: Jun 18, 2025
 *      Author: marcomiglio
 */

#include "buff_manager.h"



/*
 * Initialize system buffers (both RX buffer which stores payloads, and TX buffer implemented as a linked list).
 *
 * @param h_rx_tx*  h_rx_tx  pointer to the buffer handler (which includes handlers for both RX and TX lists).
 *
 * @return: none
 */
void init_buffers(h_rx_tx* h_rx_tx){
  // init RX FIFO
  for (uint16_t i = 0; i < BUFF_FIFO_SIZE; i++){
    h_rx_tx->h_rx[i].slot_free = true;
    h_rx_tx->h_rx[i].ll_idx = TX_IDX_EMPTY;
  }

  // init Linked List for TX sequence
  init_LL(h_rx_tx->h_tx);
}

/*
 * Remove pkt from the main buffer.
 * This automatically clears the event also in the linked list TX buffer (i.e. removing from receive FIFO
 * auotmatically removes the assocaited event in the TX buff).
 *
 * @param h_rx_tx*  h_rx_tx  pointer to the buffer handler (which includes handlers for both RX and TX lists).
 * @param uint16_t  rm_idx   index to be removed in the main buffer.
 *
 * OBS: all these flags are for debug -> in any case tha pkt is not present or has been already removed.
 *
 * @return: - RX_BUFF_SLOT_EMPTY if the slot at buff[rm_idx] is already EMPTY;
 *          - LL_IDX_IS_EMPTY if the LL is empty (pkt already removed from the TX sequence);
 *          - TX_IDX_EMPTY if this packet has no pointer to the TX sequence LL (pkt already removed from the TX sequence);
 *          - 0 otherwise;
 */
uint16_t remove_pkt(h_rx_tx* h_rx_tx, uint16_t rm_idx){
  // if the slot is already empty
  if(h_rx_tx->h_rx[rm_idx].slot_free == true) return RX_BUFF_SLOT_EMPTY;

  // This slot is occupied...

  // mark this slot as free
  h_rx_tx->h_rx[rm_idx].slot_free = true;

  uint16_t ll_idx = h_rx_tx->h_rx[rm_idx].ll_idx;
  if (ll_idx == TX_IDX_EMPTY) return TX_IDX_EMPTY;

  return remove_pkt_LL(h_rx_tx->h_tx, ll_idx);
}

/*
 * Add pkt into the main buffer.
 * This automatically creates an event also in the linked list TX buffer (i.e. adding into the receive FIFO
 * auotmatically adds the assocaited event in the TX buff).
 *
 * @param h_rx_tx*  h_rx_tx  pointer to the buffer handler (which includes handlers for both RX and TX lists).
 * @param uint16_t  add_idx  index (in the main buffer) where the new element will be places.
 *                           If add_idx = RX_BUFF_IDX_NOT_DEFINED this function will automatically spot the 1st free index for insertion.
 * @param bc_pkt*   bc       Pointer to the breadcrumb packet. The entire packet is needed (not only the LoRa payload) since based on
 *                           the RX FIFO buffer scan operation, the packet is modified (while THIS function only manages the low level
 *                           insert operation).
 *
 * @return: - RX_BUFF_FULL if the buff is FULL (notice that a FULL TX sequence implies full RX sequence);
 *          - 0 on success;
 */
uint16_t add_pkt(h_rx_tx* h_rx_tx, uint16_t add_idx, bc_pkt* bc){
  if (add_idx == RX_BUFF_IDX_NOT_DEFINED) { // if not specified determine the 1st free index in the system buffer
    add_idx = 0;
    while(h_rx_tx->h_rx[add_idx].slot_free == false) {
      add_idx+=1;
      if (add_idx == BUFF_FIFO_SIZE) return RX_BUFF_FULL;
    }
  } else {  // Ensure index within buffer boundaries
    if (add_idx >= BUFF_FIFO_SIZE) return RX_BUFF_FULL;
  }

  // now add_idx points to a free slot in the main buffer

  // add event to the TX LL
  uint16_t ll_idx = add_pkt_LL(h_rx_tx->h_tx, add_idx);

  // if FULL conclude here (LL FULL --> RX buff FULL)
  if(ll_idx == LL_IDX_IS_FULL) return RX_BUFF_FULL;

  rnode new_node = {
    .slot_free = false,
    .ll_idx    = ll_idx,
    .pkt       = *bc
  };

//  // copy payload data into the BC structure
//  memcpy(new_node.pkt.pl, bc->pl, bc->pl_len);

  h_rx_tx->h_rx[add_idx] = new_node;

  return 0;
}

/*
 * Get the next payload in the TX seqeunce.
 * The TX queue can be accessed either as a FIFO or as a LIFO by setting the corresponding field fifo_entry_point.
 * -> When accessed as a FIFO the function returns the head of the TX sequence (LL buffer), i.e. the oldest PKT.
 * -> When accessed as a LIFO, this function looks for  the first PKT with zero transmission attempts (i.e. the 1st
 *    pkt added to the buffer that has not been transmitted yet).
 * This function provides directly the BC payload (nodeID, pktID, ..., bcID sequence) of the
 * oldest pkt. The corresponding index in the RX buffer is also provided.
 *
 * @param h_rx_tx*           h_rx_tx       buffer handler;
 * @param fifo_entry_point   entry_point   TX_SEQ_ENTRY_TAIL to scan as a LIFO, TX_SEQ_ENTRY_HEAD to scan as a FIFO
 * @param uint8_t*           pyl_buff       pointer to the data buffer in which the payload will be stored;
 * @param uint16_t*          pyl_len        pointer to variable for storing actual BS-payload size (bytes);
 *
 * @return: - RX_BUFF_IDX_NOT_DEFINED set if while scanning from tail, no "new PKTs" (i.e. with zero TX attempts)
 *            were found
 *          - LL_BUFF_EMPTY if the LL is empty (notice that an empty LL doesn't mean an empty RX FIFO,
 *            maybe all the packets in the RX FIFO were already ACK);
 *          - index in the main buffer on success;
 */
uint16_t get_nextTX_pkt(h_rx_tx* h_rx_tx, fifo_entry_point entry_point, uint8_t* pyl_buff, uint8_t* pyl_len){
  uint16_t idx = RX_BUFF_IDX_NOT_DEFINED;

  if (entry_point == TX_SEQ_ENTRY_HEAD){
    // get index in the main buffer by quering the LL sequence
    idx = get_head_LL(h_rx_tx->h_tx);

    if (idx == LL_IDX_IS_EMPTY) return LL_BUFF_EMPTY;
    idx = LL_get_RXbuff_idx(h_rx_tx->h_tx, idx);

  } else if (entry_point == TX_SEQ_ENTRY_TAIL){

    // get last inserted pkt in the LL (i.e. newest pkt)
    uint16_t c_idx = get_tail_LL(h_rx_tx->h_tx);

    if (c_idx == LL_IDX_IS_EMPTY) return LL_BUFF_EMPTY;

    /*
     *  scan LL (the TX buffer) looking for the 1st inserted
     *  pkt that has not been TX yet
     */
    while (1) {
      // check if this PKT has already been TX
      uint16_t rx_idx = LL_get_RXbuff_idx(h_rx_tx->h_tx, c_idx);
      if(h_rx_tx->h_rx[rx_idx].pkt.tx_attempts > 0) break;
      idx = rx_idx;

      // up to now all the pkts are "new" ones -> find the 1st one in the sequence
      c_idx = get_prev_LL(h_rx_tx->h_tx, c_idx);

      // no remaining elements -> break here
      if (c_idx == LL_IDX_IS_HEAD) break;
    }

    // in the end -> if no "new" PKTs were present terminate here
    if (idx == RX_BUFF_IDX_NOT_DEFINED) return RX_BUFF_IDX_NOT_DEFINED;

  }

  *pyl_len = h_rx_tx->h_rx[idx].pkt.pl_len;

  // Copy into caller's buffer
  memcpy(pyl_buff, h_rx_tx->h_rx[idx].pkt.pl, *pyl_len);

  return idx;
}

/*
 * Get the last inserted payload in the TX seqeunce (i.e. newest received pkt).
 * This function provides directly the BC payload (nodeID, pktID, ..., bcID sequence) ready to be transmitted.
 * The corresponding index in the RX buffer is also provided.
 *
 * @param h_rx_tx*  h_rx_tx   buffer handler;
 * @param uint8_t*  pyl_buff  pointer to the data buffer in which the payload will be stored;
 * @param uint16_t* pyl_len   pointer to variable for storing actual BS-payload size (bytes);
 *
 * @return: - LL_BUFF_EMPTY if the LL is empty (notice that an empty LL doesn't mean an empty RX FIFO,
 *            maybe all the packets in the RX FIFO were already ACK);
 *          - index in main buffer on success;
 */
uint16_t get_lastTX_pkt(h_rx_tx* h_rx_tx, uint8_t* pyl_buff, uint16_t* pyl_len){
  // get index in the main buffer by quering the LL sequence
  uint16_t idx = get_tail_LL(h_rx_tx->h_tx);

  if (idx == LL_IDX_IS_EMPTY) return LL_BUFF_EMPTY;

  *pyl_len = h_rx_tx->h_rx[idx].pkt.pl_len;

  // Copy into caller's buffer
  memcpy(pyl_buff, h_rx_tx->h_rx[idx].pkt.pl, *pyl_len);

  return idx;
}

/*
 * Rely on this function to get priority status of the TX buffer.
 * This helps scheduling new TX events based on the priority of remaining PKTs.
 *
 * @param h_rx_tx*  h_rx_tx   buffer handler;
 *
 * @return: - LL_BUFF_EMPTY  if the LL is empty (notice that an empty LL doesn't mean an empty RX FIFO)
 *                           --> TX IRQ timer shouldn't be reactivated
 *          - TX_BUFF_PRI    If at least one "new PKT" (waiting for 1st TX) is present
 *          - TX_BUFF_NO_PRI If PKTs are waiting for retransmissions (i.e. at least on TX was already done)
 */
uint16_t get_nextTX_pri(h_rx_tx* h_rx_tx){
  // get index in the main buffer by quering the LL sequence
  uint16_t idx = get_tail_LL(h_rx_tx->h_tx);

  if (idx == LL_IDX_IS_EMPTY) return LL_BUFF_EMPTY;

  // at this point at least one element exist -> get payload in RX FIFO
  uint16_t rx_idx = LL_get_RXbuff_idx(h_rx_tx->h_tx, idx);

  return (h_rx_tx->h_rx[rx_idx].pkt.tx_attempts == 0) ? TX_BUFF_PRI : TX_BUFF_NO_PRI;
}

/*
 * Rely on this function to remove a pkt from the TX sequence when receiving an ACK.
 * This function only removes the pkt from the linked list.
 * The LL automatically updates internal links to overcome the "gap" created by the removal.
 * Remove operations are guaranteed with O(1) complexity.
 *
 * @param h_rx_tx*  h_rx_tx   buffer handler;
 * @param uint16_t   rm_idx   index in the main buffer to be removed (only its reference in the LL will be removed).
 *
 * OBS: all these flags are for debug -> in any case tha pkt is not present or has been already removed.
 *
 * @return: - TX_IDX_EMPTY   if the PKT is not present in the linked list (Already removed?);
 *          - LL_BUFF_EMPTY  if the LL is empty (notice that an empty LL doesn't mean an empty RX FIFO);
 *          - 0 on success;
 */
uint16_t tx_queue_remove(h_rx_tx* h_rx_tx, uint16_t rm_idx){

  // get idx of the associated slot in the LL sequence
  uint16_t ll_idx = h_rx_tx->h_rx[rm_idx].ll_idx;

  // pkt not present in the LL
  if (ll_idx == TX_IDX_EMPTY) return TX_IDX_EMPTY;

  // try removal on the LL
  if(remove_pkt_LL(h_rx_tx->h_tx, ll_idx) == LL_IDX_IS_EMPTY) return LL_BUFF_EMPTY;

  // At this point the reference in the TX sequence has been removed, clear the idx in the main buffer
  h_rx_tx->h_rx[rm_idx].ll_idx = TX_IDX_EMPTY;

  return 0;
}

/*
 * This function allows to add a pkt directly in the LL.
 * The LL automatically updates internal links with O(1) complexity.
 *
 * @param h_rx_tx*  h_rx_tx  buffer handler;
 * @param uint16_t  mb_idx   index in the main buffer linked to the LL (the new packet in the LL will point to this element).
 *
 * @return: - RX_BUFF_FULL  if RX buffer is full;
 *          - 0 on success;
 */
uint16_t tx_queue_add(h_rx_tx* h_rx_tx, uint16_t mb_idx){
  if (mb_idx >= BUFF_FIFO_SIZE) return RX_BUFF_FULL;

  uint16_t ll_idx = add_pkt_LL(h_rx_tx->h_tx, mb_idx);

  // if FULL conclude here (LL FULL --> RX buff FULL)
  if (ll_idx == LL_IDX_IS_FULL) return RX_BUFF_FULL;

  // link rx buffer with corresponding element in the LL
  h_rx_tx->h_rx[mb_idx].ll_idx = ll_idx;

  return 0;
}






