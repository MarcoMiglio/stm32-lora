/*
 * sys_settings.h
 *
 *  Created on: Jun 18, 2025
 *      Author: marcomiglio
 */

#ifndef INC_SYS_SETTINGS_H_
#define INC_SYS_SETTINGS_H_

//#include "main.h"

// -------------------------- LORA SETTINGS -----------------------------------

/*
 * LORA - SF:
 *
 *  SF7   0
 *  SF8   1
 *  SF9   2
 *  SF10  3
 *  SF11  4
 *  SF12  5
 */
#define LORA_SF 0

/*
 * LORA - BW:
 *
 *  BW62_5  0
 *  BW125   1
 *  BW250   2
 *  BW500   3
 */
#define LORA_BW 1

/*
 * LORA - CR:
 *
 *   CR4_5  0
 *   CR4_6  1
 *   CR4_7  2
 *   CR4_8  3
 */
#define LORA_CR 0

/*
 * LORA - TX pow in dBm (between 2 - 17 for RFM95W)
 */
#define LORA_TX_POWER  2

/*
 * LORA - FREQ. in Hz
 */
#define LORA_CH_FREQ 868000000

/*
 * LORA - Sync word for preamble detection
 */
#define LORA_SYNC_WORD 0xBA

// ----------------------------------------------------------------------------



// -------------------------- RX-TX BUFFER SETTINGS ---------------------------

/*
 * how many packets can be stored simultaneously in the RX/TX FIFO Buffer
 * Keep this value <= 65500
 */
#define BUFF_FIFO_SIZE 100

/*
 * How many BreadCrumbs are used
 */
#define BC_NUMBER 10

/*
 * BC_ID for this NODE
 */
#define MY_BC_ID 1

/*
 * Number of retransmissions before dropping a packet
 */
#define BC_TX_ATTEMPTS 3

/*
 * Sync word used to drop undesired PKTs
 */
#define SYNC_WORD_ENV 0xAA55
#define SYNC_WORD_BC  0x11AA


/*
 * Define interval boundaries for random wait time for the TX
 * of a PKT when it joins the RX FIFO
 *
 * Set MIN = MAX = 0 to skip the random wait and schedule an immediate transmission.
 */
#define MIN_WAIT_TIME_1 150u   // In milliseconds
#define MAX_WAIT_TIME_1 500u  // In milliseconds

/*
 * Define interval boundaries for random wait time for scheduling
 * retransmission attempts in case ACK is not received
 *
 * Set MIN = MAX = 0 to skip the random wait and schedule an immediate transmission.
 */
#define MIN_WAIT_TIME 200u   // In milliseconds
#define MAX_WAIT_TIME 5000u  // In milliseconds

// -----------------------------------------------------------------------------



// ----------------- RX - TX BUFFER CONSTANTS (used at runtime) ----------------

/*
 * Lebgth in bytes of each field in the BC packet
 * - Mask (1 byte)        -> bitfield to cotain different masks
 *                           - Alarm -> This is an alarm message, should take MAX priority
 *                           - RFU...
 *                           - RFU...
 *
 * - Node ID (1 byte)     -> represents the environmental node source of this pkt
 *
 * - Pkt ID  (1 bytes)    -> unique identifier for a given node ID pkt
 *
 * - BC ID   (1 byte)     -> identifiers used in the hop sequence to identify intermediate nodes
 *
 * - RSSI    (1 bytes)    -> Added on receive from an environmental node (only firs BC will add its own RSSI)
 *
 * - Payload (TODO bytes) -> Actual payload contanintg sensors readings
 *
 */
#define MASK_BYTES       1
#define SYNC_WORD_BYTES  2
#define NODE_ID_BYTES    1
#define BC_ID_BYTES      1
#define PKT_ID_BYTES     2
#define RSSI_BYTES       2
#define SENSOR_PLD_BYTES 36


/*
 * This masks are used to identify the position of each byte-field
 * in the received payload
 */
#define SYNC_WORD_POS  0
#define MASK_POS       SYNC_WORD_POS + SYNC_WORD_BYTES

#define MASK_ALARM_BIT (1 << 0)
#define RETX_ALARM_BIT (1 << 1)

#define NODE_ID_POS    MASK_POS + MASK_BYTES

#define PKT_ID_MSB_POS NODE_ID_POS + NODE_ID_BYTES
#define PKT_ID_LSB_POS NODE_ID_POS + NODE_ID_BYTES + 1

#define SMPL_DATA_POS  PKT_ID_MSB_POS + PKT_ID_BYTES

#define RSSI_POS       SMPL_DATA_POS + SENSOR_PLD_BYTES

#define BC_ID1_POS     RSSI_POS + RSSI_BYTES

/*
 * Maximum size for a received payload
 * - 2 bytes for SYNC word
 * - 1 byte for masks
 * - 1 byte node ID
 * - 2 bytes pktID
 * - 2 bytes RSSI
 * - SENSOR_PLD_BYTES bytes for sensors
 * - BC_NUMBER * (1 byte) is the maximum number of bytes used for the hopping sequence
 *   (when the pkt hops thorugh all the intermediate nodes)
 */
#define LORA_PAYLOAD_MAX_SIZE SYNC_WORD_BYTES + MASK_BYTES + NODE_ID_BYTES + PKT_ID_BYTES + RSSI_BYTES + SENSOR_PLD_BYTES + (BC_ID_BYTES * BC_NUMBER)

/*
 * Payload size receiving from end-node
 * - 2 bytes for SYNC word
 * - 1 byte for masks
 * - 1 byte node ID
 * - 2 bytes pktID
 * - SENSOR_PLD_BYTES bytes for sensors
 */
#define ENV_NODE_PYL_SIZE SYNC_WORD_BYTES + MASK_BYTES + NODE_ID_BYTES + PKT_ID_BYTES + SENSOR_PLD_BYTES

// -----------------------------------------------------------------------------

#endif /* INC_SYS_SETTINGS_H_ */
