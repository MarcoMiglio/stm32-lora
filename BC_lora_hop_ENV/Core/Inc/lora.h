/*
 * lora.h
 *
 *  Created on: May 15, 2025
 *      Author: LabMeas
 */

#ifndef INC_LORA_H_
#define INC_LORA_H_

#include <stdbool.h>
#include "stm32l4xx_hal.h"


// -------------------------- RFM generic settings ----------------------------------
#define RFM9x_VER             0x12

#define RFM95_INTERRUPT_COUNT 3 // DIO0, DIO1, DIO5 interrupt events
#define RFM95_SPI_TIMEOUT     10
#define RFM95_WAKEUP_TIMEOUT  10
#define RFM95_SEND_TIMEOUT    2000
#define RFM95_TCXO_FREQ       32000000 // RFM95 mounts 32 MHz XTAL
// ----------------------------------------------------------------------------------


// ----------------------------- LoRa specifications ---------------------------------
#define LOW_FREQ_BAND_EU868   863000000
#define HIGH_FREQ_BAND_EU868  870000000
#define MAX_EIRP_EU           16
#define LORA_DEF_SYNC_WORD    0x12
// -----------------------------------------------------------------------------------


// ---------------------------- RFM95 Registers --------------------------------------

/**
 * Register addresses
 */
#define RFM95_REGISTER_FIFO_ACCESS                     0x00
#define RFM95_REGISTER_OP_MODE                         0x01
#define RFM95_REGISTER_FR_MSB                          0x06
#define RFM95_REGISTER_FR_MID                          0x07
#define RFM95_REGISTER_FR_LSB                          0x08
#define RFM95_REGISTER_PA_CONFIG                       0x09
#define RFM95_REGISTER_LNA                             0x0C
#define RFM95_REGISTER_FIFO_ADDR_PTR                   0x0D
#define RFM95_REGISTER_FIFO_TX_BASE_ADDR               0x0E
#define RFM95_REGISTER_FIFO_RX_BASE_ADDR               0x0F
#define RFM95_REGISTER_FIFO_RX_CURR_ADDR               0x10
#define RFM95_REGISTER_IRQ_FLAGS 		               		 0x12
#define RFM95_REGISTER_FIFO_RX_BYTES_NB                0x13
#define RFM95_REGISTER_MODEM_STATUS                    0x18
#define RFM95_REGISTER_PACKET_SNR                      0x19
#define RFM95_REGISTER_PKT_RSSI                        0x1A
#define RFM95_REGISTER_MODEM_CONFIG_1                  0x1D
#define RFM95_REGISTER_MODEM_CONFIG_2                  0x1E
#define RFM95_REGISTER_SYMB_TIMEOUT_LSB                0x1F
#define RFM95_REGISTER_PREAMBLE_MSB                    0x20
#define RFM95_REGISTER_PREAMBLE_LSB                    0x21
#define RFM95_REGISTER_PAYLOAD_LENGTH                  0x22
#define RFM95_REGISTER_MAX_PAYLOAD_LENGTH              0x23
#define RFM95_REGISTER_MODEM_CONFIG_3                  0x26
#define RFM95_REGISTER_INVERT_IQ_1                     0x33
#define RFM95_REGISTER_SYNC_WORD                       0x39
#define RFM95_REGISTER_INVERT_IQ_2                     0x3B
#define RFM95_REGISTER_DIO_MAPPING_1                   0x40
#define RFM95_REGISTER_VERSION                         0x42
#define RFM95_REGISTER_PA_DAC                          0x4D


/*
 * Registers settings
 */
// Set here base addresses for data transmission and reception
#define RFM95_FIFO_RX_BASE_ADDRESS                     0x00
#define RFM95_FIFO_TX_BASE_ADDRESS                     0x00

#define RFM95_OP_MODE_SLEEP                            0x00
#define RFM95_OP_MODE_LORA_SLEEP                       0x80
#define RFM95_OP_MODE_LORA_STANDBY                     0x81
#define RFM95_OP_MODE_LORA_TX                          0x83
#define RFM95_OP_MODE_LORA_RX_CONT                     0x85
#define RFM95_OP_MODE_LORA_RX_SINGLE                   0x86
#define RFM95_PA_DAC_LOW_POWER                         0x84
#define RFM95_PA_DAC_HIGH_POWER                        0x87

#define RFM95_DIO_MAPPING_1_IRQ_FOR_TXDONE             0x40
#define RFM95_DIO_MAPPING_1_IRQ_FOR_RXDONE             0x00

#define RFM95_REG_INVERT_IQ1_RST                       0x26
#define RFM95_REG_INVERT_IQ2_RST                       0x1D

#define RFM95_INVERT_IQ_1_TX                           0x27
#define RFM95_INVERT_IQ_2_TX                           0x1d

#define RFM95_INVERT_IQ_1_RX                           0x67
#define RFM95_INVERT_IQ_2_RX                           0x19

#define RFM95_RX_DONE_MSK                              0x40
#define RFM95_PAYLOAD_CRC_ERR_MSK                      0x20

// ----------------------------------------------------------------------------------------------------


// ---------------------------------- Internal function Pointers --------------------------------------
/*
 *	implemented by user on main code
 */

// Not implemented here: function pointer to be executed after Interrupt have been enabled...
typedef void (*rfm95_on_after_interrupts_configured)();

/*
 * Get precision tick is used for generating precise timing informations for precision sleep intervals
 * and for precise opening of receive windows.
 */
typedef uint32_t (*rfm95_get_precision_tick)();


/*
 * This function allows to sleep until the target time (RFM in sleep mode, MCU in stop mode 2)
 * with error reduced to the clock drift (less than 20 ppm for STM32 boards)
 */
typedef void (*rfm95_precision_sleep_until)(uint32_t ticks_target);


/*
 * This function is used to monitor battery level to respond DevStatusReq MAC commands.
 * (Notice that it is not mandatory to implement it...).
 */
typedef uint8_t (*rfm95_get_battery_level)();

// ------------------------------------------------------------------------------------------------------



// --------------------------------- RFM95 handler / Structures -----------------------------------------

/**
 * Track RFM95 different operating modes
 */
typedef enum{
  SLEEP_MODE,
  STNBY_MODE,
  TRANSMIT_MODE,
  RXCONTIN_MODE,
  RXSINGLE_MODE
} rfm95_status;

/*
 * Interrupt events registered into the dedicated interrupt buffer
 */
typedef enum {
  RFM95_INTERRUPT_DIO0,
  RFM95_INTERRUPT_DIO1,
  RFM95_INTERRUPT_DIO5

} rfm95_interrupt_t;

/*
 * possible BW configurations:
 */
typedef enum {
	RFM95_BW62_5,
	RFM95_BW125,
	RFM95_BW250,
	RFM95_BW500
} rfm95_bw_t;

/*
 * possible CR configurations:
 */
typedef enum {
	RFM95_CR4_5,
	RFM95_CR4_6,
	RFM95_CR4_7,
	RFM95_CR4_8
} rfm95_cr_t;

/*
 * possible SF configurations:
 */
typedef enum {
	RFM95_SF7,
	RFM95_SF8,
	RFM95_SF9,
	RFM95_SF10,
	RFM95_SF11,
	RFM95_SF12
} rfm95_sf_t;

/*
 * Internal bits to control TX Power (RFM95_REGISTER_PA_CONFIG)
 */
typedef struct{
  union {
    struct {
      uint8_t output_power : 4;
      uint8_t max_power : 3;
      uint8_t pa_select : 1;
    };
    uint8_t buffer;
  };
} rfm95_register_pa_config_t;


/*
 * This structure stores all the basic configurations for the RFM95 module and the LoRa parameters
 * These values will be saved into dedicated EEPROM if save_config function is implemented.
 */
typedef struct {

  /**
   * MAGIC --> used to verify data integrity when saved into external EEPROM memory
   */
  uint16_t magic;

  /**
   * RX MAGIC --> magic keyword (included during TX-RX?)
   */
  uint16_t lora_magic;

  /**
   * SF (7 to 12)
   */
  rfm95_sf_t sf;

  /**
   * Tx power (must be in range 2 -> 17 dBm)
   */
  uint8_t tx_power;

  /**
   * Bandwidth
   */
  rfm95_bw_t bandwidth;

  /**
   * coding rate
   */
  rfm95_cr_t cr;

  /**
   * Channel frequency used for RX-TX
   */
  uint32_t channel_freq;

  /**
   * Synchronization word for raw LoRa (typically 0x12)
   */
  uint8_t sync_word;

} rfm95_config_t;


/**
 * RFM95(W) transceiver handler.
 */
typedef struct {

  /**
   *
   */
  LPTIM_HandleTypeDef *rfm_timer;

  /**
   * The handle to the SPI bus for the device.
   */
  SPI_HandleTypeDef *spi_handle;

  /**
   * The port of the NSS pin.
   */
  GPIO_TypeDef *nss_port;

  /**
   * The NSS pin.
   */
  uint16_t nss_pin;

  /**
   * The port of the RST pin.
   */
  GPIO_TypeDef *nrst_port;

  /**
   * The RST pin.
   */
  uint16_t nrst_pin;

  /**
   * The frequency of the precision tick in Hz.
   */
  uint32_t precision_tick_frequency;

  /**
   * The +/- timing drift per second in nanoseconds.
   */
  uint32_t precision_tick_drift_ns_per_s;

  /**
   * Function provided that returns a precise tick for timing critical operations.
   */
  rfm95_get_precision_tick get_precision_tick;

  /**
   * Function that provides a precise sleep until a given tick count is reached.
   */
  rfm95_precision_sleep_until precision_sleep_until;

  /**
   * Function that returns the device's battery level.
   */
  rfm95_get_battery_level get_battery_level;

  /**
   * Callback called after the interrupt functions have been properly configured;
   */
  rfm95_on_after_interrupts_configured on_after_interrupts_configured;

  /**
	* Tick values when each interrupt was called.
	*/
	volatile uint32_t interrupt_times[RFM95_INTERRUPT_COUNT];

	/**
	 * RFM95 current status:
	 */
	volatile rfm95_status rfm_status;

	/**
	 * LoRa parameters (power - SF - CR - channel frequency - bandwidth).
	 */
	rfm95_config_t config;

} rfm95_handle_t;


// ----------------------------------------------------------------------------------------------------------


// ------------------------------- User Function prototypes: ------------------------------------------------

/*
 * Public functions signature --> User should rely only on these functions!
 */

uint16_t rfm95_init(rfm95_handle_t *handle);

void reset_rfm(rfm95_handle_t *handle);

void rfm95_set_power(rfm95_handle_t *handle, uint8_t power);

void rfm95_set_SF(rfm95_handle_t *handle, rfm95_sf_t sf);

void rfm95_set_CR(rfm95_handle_t *handle, rfm95_cr_t cr);

void rfm95_set_BW(rfm95_handle_t *handle, rfm95_bw_t bw);

void rfm95_set_frequency(rfm95_handle_t *handle, uint32_t freq);

void rfm95_set_syncWord(rfm95_handle_t *handle, uint8_t syncWord);

bool rfm95_getRSSI(rfm95_handle_t *handle, int16_t *rssi);

bool rfm95_getSNR(rfm95_handle_t *handle, int8_t *snr);

bool rfm95_getModemStatus(rfm95_handle_t *handle, uint8_t *status);

bool rfm95_stdby(rfm95_handle_t *handle);

bool rfm95_send(rfm95_handle_t *handle, const uint8_t *send_data, size_t send_data_length);

bool rfm95_enter_rx_mode(rfm95_handle_t *handle);

bool rfm95_receive(rfm95_handle_t *handle, uint8_t *rx_buff, uint8_t *rx_data_length);

void rfm95_on_interrupt(rfm95_handle_t *handle, rfm95_interrupt_t interrupt);


// -------------------------------------------------------------------------------------------------------------




#endif /* INC_LORA_H_ */
