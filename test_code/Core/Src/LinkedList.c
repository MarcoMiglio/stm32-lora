/*
 * LinkedList.c
 *
 *  Created on: Jun 18, 2025
 *      Author: marcomiglio
 *
 *      This code implements a linked list on a fixed size buffer.
 */


#include <LinkedList.h>




/*
 * Initialize linked list.
 * Free node sequence is defined, s.t. insert operations can be done in O(1).
 *
 * @param LL_handler* h_LL  linked list handler.
 *
 * @return: none
 */
void init_LL(LL_handler* h_LL){
  h_LL->head = 0;
  h_LL->tail = 0;
  h_LL->ins_idx = 0;

  h_LL->ll_status = LL_EMPTY;

  // init free node sequence:
  for(uint16_t i = 0; i < BUFF_FIFO_SIZE; i++){

    h_LL->ll_buff[i].next_free_idx = i + 1;
  }

  // link last element to the flag "LL IS FULL"
  h_LL->ll_buff[BUFF_FIFO_SIZE - 1].next_free_idx = LL_IDX_IS_FULL;
}

/*
 * Add pkt to the linked list.
 * Packets are added at the internally managed index ins_idx, and automatically linked to the rest of the list.
 * Insert operations are guaranteed with O(1) complexity.
 *
 * @param LL_handler* h_LL           linked list handler.
 * @param uint16_t    idx_main_buff  index in the main RX buffer associated to this element in the LL.
 *
 * @return: - LL_IDX_IS_FULL if the LL is full;
 *          - Insertion index on success;
 */
uint16_t add_pkt_LL(LL_handler* h_LL, uint16_t idx_main_buff){
  if (h_LL->ll_status == LL_FULL) return LL_IDX_IS_FULL;

  // LL not full...

  // default node added when LL is empty
  LL_node new_node = {
    .idx_rx_buff = idx_main_buff,
    .next_element_idx = LL_IDX_IS_TAIL,
    .prev_element_idx = LL_IDX_IS_HEAD,
    .next_free_idx = 0  // used only when freeing up slots
  };

  uint16_t old_tail_idx = h_LL->tail;
  uint16_t new_tail_idx = h_LL->ins_idx;

  // if some nodes are already in the LL -> link the new one
  if (h_LL->ll_status != LL_EMPTY){
    // old tail becomes the previous node of the new tail
    new_node.prev_element_idx = old_tail_idx;

    h_LL->ll_buff[old_tail_idx].next_element_idx = new_tail_idx;
    h_LL->tail = new_tail_idx;
  }

  // update next insertion index
  h_LL->ins_idx = h_LL->ll_buff[new_tail_idx].next_free_idx;

  // update LL status after inserion
  if (h_LL->ins_idx == LL_IDX_IS_FULL) h_LL->ll_status = LL_FULL;
  else h_LL->ll_status = LL_AVAILABLE;

  h_LL->ll_buff[new_tail_idx] = new_node;

  return new_tail_idx;
}

/*
 * Remove pkt from the linked list.
 * Packets are removed and the LL automatically updates internal links to overcome the "gap" created by the removal.
 * Remove operations are guaranteed with O(1) complexity.
 *
 * @param LL_handler* h_LL   linked list handler.
 * @param uint16_t    t_idx  index in the target buffer (LL internal buffer) to be removed.
 *
 * @return: - LL_IDX_IS_EMPTY if the LL is empty;
 *          - 0 on success;
 */
uint16_t remove_pkt_LL(LL_handler* h_LL, uint16_t t_idx){
  // if LL is empty no pkt can be removed
  if (h_LL->ll_status == LL_EMPTY) return LL_IDX_IS_EMPTY;

  // at least one node exists...

  LL_node c_node = h_LL->ll_buff[t_idx];

  if ((c_node.prev_element_idx == LL_IDX_IS_HEAD) & (c_node.next_element_idx == LL_IDX_IS_TAIL)){
    // last node in the LL
    h_LL->ll_status = LL_EMPTY;

  } else {

    if (c_node.prev_element_idx == LL_IDX_IS_HEAD){         /* removing head element */

      h_LL->head = c_node.next_element_idx;
      h_LL->ll_buff[h_LL->head].prev_element_idx = LL_IDX_IS_HEAD;

    } else if (c_node.next_element_idx == LL_IDX_IS_TAIL){  /* removing tail element */

      h_LL->tail = c_node.prev_element_idx;
      h_LL->ll_buff[h_LL->tail].next_element_idx = LL_IDX_IS_TAIL;

    } else {                                                /* removing intermediate node */

      h_LL->ll_buff[c_node.prev_element_idx].next_element_idx = c_node.next_element_idx;
      h_LL->ll_buff[c_node.next_element_idx].prev_element_idx = c_node.prev_element_idx;
    }
  }

  // Update free slot sequence for O(1) isnertions
  h_LL->ll_buff[t_idx].next_free_idx = h_LL->ins_idx;
  h_LL->ins_idx = t_idx;

  return 0;
}

/*
 * Get head element index of the linked list buffer.
 * The returned index refers to the LL buffer.
 *
 * @param LL_handler* h_LL   linked list handler.
 *
 * @return: - LL_IDX_IS_EMPTY if the LL is empty;
 *          - index in main buffer on success;
 */
uint16_t get_head_LL(LL_handler* h_LL){
  // return index in main buffer (RX FIFO) marked as head of the sequence
  if (h_LL->ll_status != LL_EMPTY) return h_LL->head;

  // if LL is empty, return false index
  return LL_IDX_IS_EMPTY;
}

/*
 * Get idx of the tail of the linked list.
 * The returned index refers to the element in the TX buffer designated as the tail of the transmission sequence
 * (i.e. the latest received pkt, "newest" in the RX FIFO).
 * Access is guaranteed in O(1) complexity.
 *
 * @param LL_handler* h_LL   linked list handler.
 *
 * @return: - LL_IDX_IS_EMPTY if the LL is empty;
 *          - index in main buffer on success;
 */
uint16_t get_tail_LL(LL_handler* h_LL){
  // return index in main buffer (RX FIFO) marked as tail of the sequence
  if (h_LL->ll_status != LL_EMPTY) return h_LL->tail;

  // if LL is empty, return false index
  return LL_IDX_IS_EMPTY;
}

/*
 * Get RX buffer idx associated to this entry in the LL.
 *
 * @param LL_handler* h_LL   linked list handler.
 *
 * @return: - LL_IDX_IS_EMPTY if the LL is empty;
 *          - index in main buffer on success;
 */
uint16_t LL_get_RXbuff_idx(LL_handler* h_LL, uint16_t idx){
  // return index in main buffer (RX FIFO) corresponding to this LL element
  return h_LL->ll_buff[idx].idx_rx_buff;
}

/*
 * Get pkt after the one i am pointing to in the LL
 * The returned index refers to the element in the Linked list buffer (i.e. TX buffer)
 *
 * @param LL_handler* h_LL       linked list handler.
 * @param uint16_t    curr_idx   Index of the element i am currently pointing to
 *
 * @return: - LL_IDX_IS_EMPTY if the LL is empty;
 *          - index of next element otherwise.
 *            (OBS!!! if CURR element is tail, next element is LL_IDX_IS_TAIL.
 */
uint16_t get_next_LL(LL_handler* h_LL, uint16_t curr_idx){
  // return index of the next element in the LL
  if (h_LL->ll_status != LL_EMPTY) return h_LL->ll_buff[curr_idx].next_element_idx;

  // if LL is empty, return false index
  return LL_IDX_IS_EMPTY;
}

/*
 * Get pkt before the one i am pointing to in the LL
 * The returned index refers to the element in the Linked list buffer (i.e. TX buffer)
 *
 * @param LL_handler* h_LL       linked list handler.
 * @param uint16_t    curr_idx   Index of the element i am currently pointing to
 *
 * @return: - LL_IDX_IS_EMPTY if the LL is empty;
 *          - index of next element otherwise.
 *            (OBS!!! if CURR element is head, prev element is LL_IDX_IS_HEAD.
 */
uint16_t get_prev_LL(LL_handler* h_LL, uint16_t curr_idx){
  // return index of the next element in the LL
  if (h_LL->ll_status != LL_EMPTY) return h_LL->ll_buff[curr_idx].prev_element_idx;

  // if LL is empty, return false index
  return LL_IDX_IS_EMPTY;
}




