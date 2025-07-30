/*
 * buff_manager.h
 *
 *  Created on: Jun 18, 2025
 *      Author: marcomiglio
 */

#ifndef INC_BUFF_MANAGER_H_
#define INC_BUFF_MANAGER_H_


#include "string.h"
#include "stdbool.h"
#include "LinkedList.h"



// ---------------------------------------- INTERNAL DEFINES ----------------------------------------------

/*
 * Additional indexes used to setup status flags. They are defined on purpose "outside of array boundaries", in this way
 * they cannot coincide with a valid buffer index (no ambiguity).
 *
 * (Index offsets start from +10 since values from 0->9 are reserved for LL flags (avoid unintended conflicts).
 */
#define RX_BUFF_FULL             (BUFF_FIFO_SIZE + 10)   // Main RX FIFO is full
#define RX_BUFF_SLOT_EMPTY       (BUFF_FIFO_SIZE + 11)   // Flag returned when accessing an empty slot
#define TX_IDX_EMPTY             (BUFF_FIFO_SIZE + 12)   // The pkt in the main buffer is not linked to any index in the LL (already ACK?)
#define RX_BUFF_IDX_NOT_DEFINED  (BUFF_FIFO_SIZE + 13)   // When adding a pkt to the RX FIFO, if insert idx = RX_BUFF_IDX_NOT_DEFINED
                                                         // the 1st avaialble slot will be identified and used for insertion.
#define LL_BUFF_EMPTY            (BUFF_FIFO_SIZE + 14)   // LL buff is EMPTY (This doesn't mean that RX-BUFF is also EMPTY!)
#define TX_BUFF_NO_PRI           (BUFF_FIFO_SIZE + 15)   // LL buff has no "new PKTs" -> only retransmissions
#define TX_BUFF_PRI              (BUFF_FIFO_SIZE + 16)   // LL buff has at least one "new PKT" (waiting for 1st TX)
// --------------------------------------------------------------------------------------------------------


// --------------------------------------- STRUCTURES/TYPEDEFs --------------------------------------------

/*
 * Define how TX QUEUE is scan
 *  -> from TAIL gives priority to new pkts
 *  -> from HEAD looks for retransmissions
 */
typedef enum{
  TX_SEQ_ENTRY_TAIL,
  TX_SEQ_ENTRY_HEAD
} fifo_entry_point;

/*
 * Breadcrumb structure
 */
typedef struct{
  uint8_t tx_attempts;  // TX attempts performed for this specific payload

  bool ack;             // Track ACKs for this packet

  uint8_t nodeID;       // Node ID for this packet

  uint16_t pktID;        // pkt ID to identify this specific payload

  uint8_t rx_bcID;      // ID of the breadcrumb that has received this packet from the environmental node

  uint8_t tx_bcID;      // ID of the breadcrumb that has transmitted this packet to this node

  uint8_t pl_len;       // Number of bytes of the entire packet

  uint8_t pl[LORA_PAYLOAD_MAX_SIZE]; // Pointer to the entire payload received (Buffer size fixed to avoid dynamic memory allocation)
} bc_pkt;

/*
 * Slot inside the receive buffer containing all the necessary informations
 */
typedef struct{
  bool slot_free;   // This slot can be used to insert a new BC pkt

  uint16_t ll_idx;  // index (in the LL) of the LL node corresponding to this payload -> LL Tracks the TX sequence

  bc_pkt pkt;       // BC pkt stored in this slot
} rnode;

/*
 * RX Buffer handler. User should rely on this strcuture for poper system functionality:
 * it automatically handles add/remove operations as well as tracking of the packets transmission sequence.
 */
typedef struct{
  LL_handler* h_tx;  // Pointer to LL handler used to track packets transmission sequence

  rnode* h_rx;       // Pointer to the buffer of BC packets -> i.e. the receieve buffer
} h_rx_tx;

// --------------------------------------------------------------------------------------------------------


// ---------------------------------------- Function prototypes -------------------------------------------

void init_buffers(h_rx_tx* h_rx_tx);

uint16_t remove_pkt(h_rx_tx* h_rx_tx, uint16_t rm_idx);

uint16_t add_pkt(h_rx_tx* h_rx_tx, uint16_t add_idx, bc_pkt* bc);

uint16_t get_nextTX_pkt(h_rx_tx* h_rx_tx, fifo_entry_point entry_point, uint8_t* pyl_buff, uint8_t* pyl_len);

uint16_t get_lastTX_pkt(h_rx_tx* h_rx_tx, uint8_t* pyl_buff, uint16_t* pyl_len);

uint16_t get_nextTX_pri(h_rx_tx* h_rx_tx);

uint16_t tx_queue_remove(h_rx_tx* h_rx_tx, uint16_t rm_idx);

uint16_t tx_queue_add(h_rx_tx* h_rx_tx, uint16_t add_idx);


// --------------------------------------------------------------------------------------------------------


#endif /* INC_BUFF_MANAGER_H_ */
