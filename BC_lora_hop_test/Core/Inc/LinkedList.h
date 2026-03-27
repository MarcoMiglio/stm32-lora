/*
 * LinkedList.h
 *
 *  Created on: Jun 18, 2025
 *      Author: marcomiglio
 */

#ifndef INC_LINKEDLIST_H_
#define INC_LINKEDLIST_H_


#include "stdint.h"
#include "sys_settings.h"


// ------------------------------------- INTERNAL DEFINES -------------------------------------------------
/* User should not modify this internal variables, but rely on the configuration file "sys_settings.h" */

/*
 * These are virtual indexes used to track LL current status. They cannot be used neither to store LL_nodes
 * nor to read back valid values (They are simple pointers to maintain the head/tail index boundaries).
 */
#define LL_IDX_IS_FULL  (BUFF_FIFO_SIZE + 0)
#define LL_IDX_IS_EMPTY (BUFF_FIFO_SIZE + 1)
#define LL_IDX_IS_HEAD  (BUFF_FIFO_SIZE + 2)
#define LL_IDX_IS_TAIL  (BUFF_FIFO_SIZE + 3)

// --------------------------------------------------------------------------------------------------------


// ------------------------------------ STRUCTURES/TYPEDEFs -----------------------------------------------

/*
 * Node inside the linked list
 */
typedef struct{
  uint16_t idx_rx_buff;       // track element in the main buffer for O(1) accesses

  uint16_t next_element_idx;  // next element in the linked list

  uint16_t prev_element_idx;  // previous element in the linked list

  uint16_t next_free_idx;     // track free slots for O(1) insertions
} LL_node;

/*
 * Linked list possible states (FULL, EMPTY, AVAILABLE)
 */
typedef enum {
  LL_FULL,
  LL_EMPTY,
  LL_AVAILABLE
} LL_status;

/*
 * Linked list handler -> used to track LL state and perform add/remove/get operations
 */
typedef struct{
  uint16_t head;          // track LL head for O(1) extraction (head = 1st in element)

  uint16_t tail;          // track LL tail for O(1) insertions --> tail needed to link new inserted elements with the old tail

  uint16_t ins_idx;       // track LL insertion point --> all the other free indexes can be accessed starting from this one (scan next_free_idx)

  LL_node* ll_buff;       // pointer to the LL_node[BUFF_SIZE] buffer that contains the LL nodes

  LL_status ll_status;    // LL states
} LL_handler;

// --------------------------------------------------------------------------------------------------------


// ----------------------------------------- Function prototypes ------------------------------------------

void init_LL(LL_handler* h_LL);

uint16_t add_pkt_LL(LL_handler* h_LL, uint16_t idx_main_buff);

uint16_t remove_pkt_LL(LL_handler* h_LL, uint16_t t_idx);

uint16_t get_head_LL(LL_handler* h_LL);

uint16_t LL_get_RXbuff_idx(LL_handler* h_LL, uint16_t idx);

uint16_t get_tail_LL(LL_handler* h_LL);

uint16_t get_next_LL(LL_handler* h_LL, uint16_t curr_idx);

uint16_t get_prev_LL(LL_handler* h_LL, uint16_t curr_idx);

// ---------------------------------------------------------------------------------------------------------

#endif /* INC_LINKEDLIST_H_ */
