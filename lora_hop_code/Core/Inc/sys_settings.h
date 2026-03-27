/*
 * sys_settings.h
 *
 *  Created on: Jun 18, 2025
 *      Author: marcomiglio
 */

#ifndef INC_SYS_SETTINGS_H_
#define INC_SYS_SETTINGS_H_

#include "main.h"

// --------------------- DEBUG PRINTF ENABLE/DISABLE --------------------------

#define DEBUG_PRINT_ON 0

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
#define MY_BC_ID 2

/*
 * Number of retransmissions before dropping a packet
 */
#define BC_TX_ATTEMPTS 3

/*
 * TX limit for alarm PKTs
 */
#define ALARM_MAX_TX_ATTEMPTS 10

/*
 * Sync word used to drop undesired PKTs
 */
#define SYNC_WORD_ENV 0xAA55
#define SYNC_WORD_BC  0x11AA

/*[NEW PKT WAIT --> VALID FOR BOTH NORMAL - ALARM PKTs]
 *
 * Define interval boundaries for random wait time for the TX
 * of a PKT when it joins the RX FIFO (new PKT just received)
 *
 * Set MIN = MAX = 0 to skip the random wait and schedule an immediate transmission.
 */
#define MIN_WAIT_TIME_NEW 0u  // In milliseconds
#define MAX_WAIT_TIME_NEW 0u  // In milliseconds

/*[ALRM ACK WAIT --> ONLY FOR ALARM PKTs]
 *
 * Random wait time before sending ACK on request during
 * ALARM operation (ACK msg for a node behind me)
 *
 * Set MIN = MAX = 0 to skip the random wait and schedule an immediate transmission.
 */
#define MIN_WAIT_TIME_ALRM_ACK 0u   // In milliseconds
#define MAX_WAIT_TIME_ALRM_ACK 0u   // In milliseconds

/*[ReTX SHORT WAIT --> BOTH NORMAL - ALARM PKTs]
 *
 * Define interval boundaries for random wait time for the TX
 * of a PKT when "new PKTs" (i.e. with zero TX attempts) are
 * waiting in queue
 *
 * Set MIN = MAX = 0 to skip the random wait and schedule an immediate transmission.
 */
#define MIN_WAIT_TIME_SHORT 200u  // In milliseconds
#define MAX_WAIT_TIME_SHORT 500u  // In milliseconds

/*[ReTX LONG WAIT --> ONLY FOR NORMAL PKTs]
 *
 * Define interval boundaries for random wait time for scheduling
 * retransmission attempts in case ACK is not received
 *
 * Set MIN = MAX = 0 to skip the random wait and schedule an immediate transmission.
 */
#define MIN_WAIT_TIME_LONG 200u   // In milliseconds
#define MAX_WAIT_TIME_LONG 5000u  // In milliseconds

/*[ReTX LONG WAIT --> ONLY FOR ALARM PKTs]
 *
 * Define interval boundaries for random wait time for scheduling
 * retransmission attempts for NACK ALARM PKTs
 *
 * Set MIN = MAX = 0 to skip the random wait and schedule an immediate transmission.
 */
#define MIN_WAIT_ALRM_RETX 2000u   // In milliseconds
#define MAX_WAIT_ALRM_RETX 5000u  // In milliseconds

/*
 * ALARM timeout CYCLES defines how many RTC wkup cycles (each lasts
 * ALARM_TIMEOUT_MS ms) the system has to wait in ALARM STDBY MODE before clearing
 * the buffer and returning to normal operating mode.
 *
 * This represents how much time the system has to wait without receiving
 * any ALARM event before clearing ALARM RX FIFO, and return to normal conditions.
 *
 * Total time (ms) = TIMEOUT_CYCLES * 30000ms
 *
 * (programmable in steps of 30 s)
 */
#define ALARM_TIMEOUT_CYCLES  2
#define ALARM_TIMEOUT_MS      30 * 1000 // In ms (do not modify!)

// -----------------------------------------------------------------------------


// --------------------------- RTC VARIABLES (RFU) -----------------------------

/*
 * Modify these 3 if needed
 */
#define START_DAY    0x01
#define START_MONTH  0x08
#define START_YEAR   0x25

/*
 * Constants that shouldn't be accessed
 */

// BCD to Decimal macro -> Convert RTC format into deciamal values
#define BCD_TO_DEC(x)    (((x) >> 4) * 10 + ((x) & 0x0F))

#define START_DAY_DEC     BCD_TO_DEC(START_DAY)
#define START_MONTH_DEC   BCD_TO_DEC(START_MONTH)
#define START_YEAR_DEC    BCD_TO_DEC(START_YEAR)

#define MS_PER_S     1000
#define S_PER_HOUR   3600
#define S_PER_MIN    60
#define M_PER_HOUR   60
#define H_PER_DAY    24

#define MS_PER_DAY   ((H_PER_DAY) * (S_PER_HOUR) * (MS_PER_S))
// -----------------------------------------------------------------------------



// ----------------- RX - TX BUFFER CONSTANTS (used at runtime) ----------------

/*
 * Lebgth in bytes of each field in the BC packet
 * - Sync word (2 bytes)  -> 0xAA55 for uplinks between ENV -> BC node
 *                           0x11AA for communication NODE -> NODE
 *
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

// MASK BIT-FIELD
#define ALARM_BIT_POS   0
#define ACK_BIT_POS     1
#define RETX_BIT_POS    2

#define MASK_ALARM_BIT  (1 << ALARM_BIT_POS)
#define MASK_ALARM_ACK  (1 << ACK_BIT_POS)
#define MASK_RETX_BIT   (1 << RETX_BIT_POS)

#define NODE_ID_POS    MASK_POS + MASK_BYTES

#define PKT_ID_MSB_POS NODE_ID_POS + NODE_ID_BYTES
#define PKT_ID_LSB_POS NODE_ID_POS + NODE_ID_BYTES + 1

#define SMPL_DATA_POS  PKT_ID_MSB_POS + PKT_ID_BYTES

#define RSSI_POS       SMPL_DATA_POS + SENSOR_PLD_BYTES

#define BC_ID1_POS     RSSI_POS + RSSI_BYTES

/*
 * Maximum size for a received payload
 * - 2 bytes for sync word
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
 * - 2 bytes for sync word
 * - 1 byte for masks
 * - 1 byte node ID
 * - 2 bytes pktID
 * - SENSOR_PLD_BYTES bytes for sensors
 */
#define ENV_NODE_PYL_SIZE SYNC_WORD_BYTES + MASK_BYTES + NODE_ID_BYTES + PKT_ID_BYTES + SENSOR_PLD_BYTES

/*
 * Minimum payload size receiving from BC-node
 * - 2 bytes for sync word
 * - 1 byte for masks
 * - 1 byte node ID
 * - 2 bytes pktID
 * - SENSOR_PLD_BYTES bytes for sensors
 * - 2 bytes for RSSI
 * - at least one byte for bcID
 */
#define BC_NODE_MIN_PYL_SIZE SYNC_WORD_BYTES + MASK_BYTES + NODE_ID_BYTES + PKT_ID_BYTES + SENSOR_PLD_BYTES + RSSI_BYTES + BC_ID_BYTES

// -----------------------------------------------------------------------------

#endif /* INC_SYS_SETTINGS_H_ */
