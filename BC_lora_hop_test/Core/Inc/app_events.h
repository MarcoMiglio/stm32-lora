/*
 * app_events.h
 *
 *  Created on: Jul 16, 2025
 *      Author: marcomiglio
 */

#ifndef INC_APP_EVENTS_H_
#define INC_APP_EVENTS_H_

#include "stm32l4xx_hal.h"
#include "buff_manager.h"
#include "sys_settings.h"
#include "lora.h"


// --------------------------------------- STRUCTURES/TYPEDEFs --------------------------------------------

typedef struct {
    uint8_t err_flags;    // each bit is an error flag
    uint8_t status_flags; // each bit is a status flag
} events_flags;

#define EVT_RFM_SPI_ERR      (1 << 0)
#define EVT_RFM_RX_ERR       (1 << 1)
#define EVT_RX_FIFO_FULL     (1 << 2)
#define EVT_BAD_PKT_FORMAT   (1 << 3)

#define EVT_SCHEDULE_PRI_TX  (1 << 0)
#define EVT_SCHEDULE_TX      (1 << 1)
#define EVT_RFM_MODEM_RX     (1 << 2)
#define EVT_TX_FIFO_EMPTY    (1 << 3)


// --------------------------------------------------------------------------------------------------------


// ---------------------------------------- Function prototypes -------------------------------------------

events_flags on_rx_event(rfm95_handle_t* h_rfm, h_rx_tx* h_fifo);

events_flags on_tx_event(rfm95_handle_t* h_rfm, h_rx_tx* h_fifo);

events_flags process_bcNode_up(h_rx_tx* h_fifo, bc_pkt* rx_pkt);

events_flags process_envNode_up(h_rx_tx* h_fifo, bc_pkt* rx_pkt, int16_t rssi);

events_flags process_bcNode_ack(h_rx_tx* h_fifo, bc_pkt* rx_pkt);

// --------------------------------------------------------------------------------------------------------

#endif /* INC_APP_EVENTS_H_ */
