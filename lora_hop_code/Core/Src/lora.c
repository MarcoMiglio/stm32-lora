/*
 * lora.c
 *
 *  Created on: May 15, 2025
 *      Author: LabMeas
 */

#include "lora.h"
#include "stdbool.h"
#include "string.h"

// ------------------------------- STATIC MEMBERS --------------------------------------
/*
 *  (internally used, USER SHOULD NEVER RELY ON THIS STRUCTURES!)
 */

/*
 * TX power configuration register
 */
static rfm95_register_pa_config_t pa_config;

/*
 * Lookup table for BW (in Hz)
 */
static const uint32_t RFM95_BW_HZ[] = {
    62500,    // RFM95_BW62_5
    125000,   // RFM95_BW125
    250000,   // RFM95_BW250
    500000    // RFM95_BW500
};

/*
 * Lookup table for BW (bits)
 */
static const uint32_t RFM95_BW_BIN[] = {
    0b0110,   // RFM95_BW62_5
    0b0111,   // RFM95_BW125
    0b1000,   // RFM95_BW250
    0b1001    // RFM95_BW500
};

/*
 * Lookup table for SF settings
 */
static const uint8_t RFM95_SF[] = {
    7,    // RFM95_SF7
    8,    // RFM95_SF8
    9,    // RFM95_SF9
    10,   // RFM95_SF10
		11,   // RFM95_SF11
		12		// RFM95_SF12
};

/*
 * Lookup table for CR settings
 */
static const uint8_t RFM95_CR[] = {
    1,    // RFM95_CR_4_5
    2,    // RFM95_CR_4_6
    3,    // RFM95_CR_4_7
    4,    // RFM95_CR_4_8
};


// -------------------------------------------------------------------------------------


// --------------------------------------------- STATIC FUNCTIONS -----------------------------------------------------
/*
 *  (internally used, USER SHOULD NEVER RELY ON THIS FUNCTIONS!)
 */

/* Read RFM95 internal register
 *
 * @param *handle pointer to handle structure containing rfm95 configurations (refer to .h file for additional details
 *                about the structure).
 * @param reg     rfm95_register_t specifier for one of the RFM95 internal registers
 * @param *buffer pointer to data buffer to be written into the register
 * @param length  size_t specifying amount of bytes to be written
 *
 * @return: false if an SPI error occurred, true otherwise
 */
static bool read_register(rfm95_handle_t *handle, uint8_t reg, uint8_t *buffer, size_t length) {
  HAL_GPIO_WritePin(handle->nss_port, handle->nss_pin, GPIO_PIN_RESET);

  uint8_t transmit_buffer = reg & 0x7fu;

  if (HAL_SPI_Transmit(handle->spi_handle, &transmit_buffer, 1, RFM95_SPI_TIMEOUT) != HAL_OK) {
    return false;
  }

  if (HAL_SPI_Receive(handle->spi_handle, buffer, length, RFM95_SPI_TIMEOUT) != HAL_OK) {
    return false;
  }

  HAL_GPIO_WritePin(handle->nss_port, handle->nss_pin, GPIO_PIN_SET);

  return true;
}

/* Write to RFM95 internal register
 *
 * @param *handle pointer to handle structure containing rfm95 configurations (refer to .h file for additional details
 *                about the structure).
 * @param reg     rfm95_register_t specifier for one of the RFM95 internal registers
 * @param value   uint8_t specifying byte to be written into the register
 *
 *
 * @return: false if an SPI error occurred, true otherwise
 */
static bool write_register(rfm95_handle_t *handle, uint8_t reg, uint8_t value) {
  HAL_GPIO_WritePin(handle->nss_port, handle->nss_pin, GPIO_PIN_RESET);

  uint8_t transmit_buffer[2] = {(reg | 0x80u), value};

  if (HAL_SPI_Transmit(handle->spi_handle, transmit_buffer, 2, RFM95_SPI_TIMEOUT) != HAL_OK) {
    return false;
  }

  HAL_GPIO_WritePin(handle->nss_port, handle->nss_pin, GPIO_PIN_SET);

  return true;
}

/* modify RFM registers to setup current power configuration.
 * (OBS: Over current protection "OCP" is not implemented here. The default value of 100 mA is sufficient
 * for output power levels up to 17 dBm).
 *
 * @param *handle pointer to handle structure containing rfm95 configurations (refer to .h file for additional details
 *                about the structure).
 *
 * @return: false if an error occurred, true otherwise
 */
static bool rfm95_modify_power(rfm95_handle_t *handle){
	memset(&pa_config, 0, sizeof(pa_config));
	uint8_t pa_dac_config = 0;

	uint8_t power = handle->config.tx_power;

	if (power >= 2 && power <= 17) {
		pa_config.max_power = 7;
		pa_config.pa_select = 1;
		pa_config.output_power = (power - 2);
		pa_dac_config = RFM95_PA_DAC_LOW_POWER;

	} else if (power == 20) {
		pa_config.max_power = 7;
		pa_config.pa_select = 1;
		pa_config.output_power = 15;
		pa_dac_config = RFM95_PA_DAC_HIGH_POWER;
	}

	if (!write_register(handle, RFM95_REGISTER_PA_CONFIG, pa_config.buffer)) return false;
	if (!write_register(handle, RFM95_REGISTER_PA_DAC, pa_dac_config)) return false;

	return true;
}

/* modify RFM registers to setup current SF configuration
 *
 * @param *handle pointer to handle structure containing rfm95 configurations (refer to .h file for additional details
 *                about the structure).
 *
 * @return: false if an error occurred, true otherwise
 */
static bool rfm95_modify_SF(rfm95_handle_t *handle){
	uint8_t  sf = RFM95_SF[handle->config.sf];

	// Configure modem SF (depends on user configuration + DR offset):
	uint8_t sf_bits = (sf << 4) | 0x04; // set SF + CRC enable
	if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_2, sf_bits)) return false;

	// AGC on (suggested in application note), LDR optimization only for Ts > 16 ms
	uint32_t bw = RFM95_BW_HZ[handle->config.bandwidth];

	// compute symbol time (in ms) based on current settings:
	float curr_ts = 1e3*(((float) (1<<sf)) / bw);
	uint8_t LDRoptimize = curr_ts >= 16 ? 0x0C : 0x04;
	if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_3, LDRoptimize)) return false;

	return true;
}

/* modify RFM registers to setup current CR and BW configuration
 *
 * @param *handle pointer to handle structure containing rfm95 configurations (refer to .h file for additional details
 *                about the structure).
 *
 * @return: false if an error occurred, true otherwise
 */
static bool rfm95_modify_CR_BW(rfm95_handle_t *handle){
	uint8_t  cr = RFM95_CR[handle->config.cr];
	uint32_t bw = RFM95_BW_BIN[handle->config.bandwidth];

	uint8_t data = (bw << 4) | (cr << 1);
	if(!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_1, data)) return false;

	return true;
}

/* modify RFM registers to setup current channel frequency configuration
 *
 * @param *handle pointer to handle structure containing rfm95 configurations (refer to .h file for additional details
 *                about the structure).
 *
 * @return: false if an error occurred, true otherwise
 */
static bool rfm95_modify_frequency(rfm95_handle_t *handle){
  uint32_t frequency = handle->config.channel_freq;

  // FQ = (FRF * 32 Mhz) / (2 ^ 19)
  uint64_t frf = ((uint64_t)frequency << 19) / RFM95_TCXO_FREQ;

  if (!write_register(handle, RFM95_REGISTER_FR_MSB, (uint8_t)(frf >> 16))) return false;
  if (!write_register(handle, RFM95_REGISTER_FR_MID, (uint8_t)(frf >> 8))) return false;
  if (!write_register(handle, RFM95_REGISTER_FR_LSB, (uint8_t)(frf >> 0))) return false;

	return true;
}

/* modify RFM registers to setup current synchronization word
 *
 * @param *handle pointer to handle structure containing rfm95 configurations (refer to .h file for additional details
 *                about the structure).
 *
 * @return: false if an error occurred, true otherwise
 */
static bool rfm95_modify_syncWord(rfm95_handle_t *handle){
  if (!write_register(handle, RFM95_REGISTER_SYNC_WORD, handle->config.sync_word)) return false;

	return true;
}

/* This function is used during initialization process.
 * If no configurations are either stored (external EEPROM) or set by the user before calling this function,
 * the default ones are used:
 * - TX SF = 7
 * - TX power = 1 dBm
 * - CR = 4/5
 * - BW = 125 kHz
 * - synchWord = 0x12
 * - channel Freq. = 868 MHz
 *
 * @param *handle rfm95_handle_t structure containing RFM95 configurations and function pointers.
 *
 * @return void.
 */
static void config_load_default(rfm95_handle_t *handle){
  if (handle->config.sf == 0) 					rfm95_set_SF(handle, RFM95_SF7);
  if (handle->config.cr == 0) 					rfm95_set_CR(handle, RFM95_CR4_5);
  if (handle->config.tx_power == 0)  		rfm95_set_power(handle, 14);
  if (handle->config.bandwidth == 0) 		rfm95_set_BW(handle, RFM95_BW125);
  if (handle->config.sync_word == 0) 		rfm95_set_syncWord(handle, LORA_DEF_SYNC_WORD);
  if (handle->config.channel_freq == 0) rfm95_set_frequency(handle, 868000000);

  return;
}

/* Wait for a single interrupt event. If exceeds the timeout threshold, the while execution
 * is interrupted.
 *
 * This function is used for interrupts on DIO5 pin (wait for the rfm95 module to be ready after changing operating mode)
 * and on DIO0 pin during TX (wait for TX done event).
 *
 * @param *handle    rfm95_handle_t structure containing RFM95 configurations
 * @param interrupt  rfm95_interrupt_t defining the interrupt event to wait for (DIO0, DIO1, DIO5 supported in this version)
 * @param timeout_ms uint32_t defining the maximum timeout in ms.
 *
 * @return true if an interrupt is received within the specified timeout, false otherwise
 */
static bool wait_for_irq(rfm95_handle_t *handle, rfm95_interrupt_t interrupt, uint32_t timeout_ms) {
  uint32_t timeout_tick = handle->get_precision_tick() + timeout_ms * handle->precision_tick_frequency / 1000;

  while (handle->interrupt_times[interrupt] == 0) {
    if (handle->get_precision_tick() >= timeout_tick) {
      return false;
    }
  }

  return true;
}





// -------------------------------- USER PUBLIC FUNCTIONS -------------------------------

uint16_t rfm95_init(rfm95_handle_t *handle){


  reset_rfm(handle);

  // setup default configurations:
  config_load_default(handle);

  // Check for correct version.
  uint8_t version;
  if (!read_register(handle, RFM95_REGISTER_VERSION, &version, 1)) return false;
  if (version != RFM9x_VER) return false;

  // Module must be placed in sleep mode before switching to LoRa.
  if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_OP_MODE_SLEEP)) return false;
  if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_OP_MODE_LORA_SLEEP)) return false;

  // Default interrupt configuration, must be done to prevent DIO5 clock interrupts at 1Mhz
  if (!write_register(handle, RFM95_REGISTER_DIO_MAPPING_1, RFM95_DIO_MAPPING_1_IRQ_FOR_RXDONE)) return false;

  if (handle->on_after_interrupts_configured != NULL) {
    handle->on_after_interrupts_configured();
  }

  // Set LNA to the highest gain with 150% boost (suggested in AN)
  if (!write_register(handle, RFM95_REGISTER_LNA, 0x23)) return false;

  // Preamble set to 10 + 4.25 = 14.25 symbols
  if (!write_register(handle, RFM95_REGISTER_PREAMBLE_MSB, 0x00)) return false;
  if (!write_register(handle, RFM95_REGISTER_PREAMBLE_LSB, 0x0A)) return false;

  // Set up TX and RX FIFO base addresses.
  if (!write_register(handle, RFM95_REGISTER_FIFO_TX_BASE_ADDR, RFM95_FIFO_TX_BASE_ADDRESS)) return false;
  if (!write_register(handle, RFM95_REGISTER_FIFO_RX_BASE_ADDR, RFM95_FIFO_RX_BASE_ADDRESS)) return false;

  // Maximum payload length of the RFM95 is 0xFF.
  if (!write_register(handle, RFM95_REGISTER_MAX_PAYLOAD_LENGTH, 0xFF)) return false;

  // Let module sleep after initialization.
  if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_OP_MODE_LORA_SLEEP)) return false;
  handle->rfm_status = SLEEP_MODE;

  return true;
}

/* This function drives the RFM95 rest pin low to force an hardware reset.
 *
 * @param *handle rfm95_handle_t structure containing RFM95 configurations and function pointers.
 *
 * @return void.
 */
void reset_rfm(rfm95_handle_t *handle){
  HAL_GPIO_WritePin(handle->nrst_port, handle->nrst_pin, GPIO_PIN_RESET);
  HAL_Delay(1); // 0.1ms would theoretically be enough
  HAL_GPIO_WritePin(handle->nrst_port, handle->nrst_pin, GPIO_PIN_SET);
  HAL_Delay(5);

  return;
}

/* Use this function to set RFM TX power.
 * This function only updates the value in the handler.config structure.
 * The modification will be effective in the RFM registers only during the next read/transmit call.
 * This prevents to modify RFM status while in transmission/reception state.
 *
 * @param *handle rfm95_handle_t structure containing RFM95 configurations and function pointers.
 * @param uint8_t power          must be wihin the range 2 dbm - 17 dBm.
 *
 * @return void.
 */
void rfm95_set_power(rfm95_handle_t *handle, uint8_t power){
	uint8_t pw = power;
	if (power < 2)  pw = 2;
	if (power > 17) pw = 17;

	handle->config.tx_power = pw;

	return;
}

/* Use this function to set RFM SF.
 * This function only updates the value in the handler.config structure.
 * The modification will be effective in the RFM registers only during the next read/transmit call.
 * This prevents to modify RFM status while in transmission/reception state.
 *
 * @param *handle    rfm95_handle_t structure containing RFM95 configurations and function pointers.
 * @param rfm95_sf_t sf             must be within 7 - 12 (rely on the dedicated rfm95_sf_t structure, do not manually write).
 *
 * @return void.
 */
void rfm95_set_SF(rfm95_handle_t *handle, rfm95_sf_t sf){
	handle->config.sf = sf;
	return;
}

/* Use this function to set RFM CR.
 * This function only updates the value in the handler.config structure.
 * The modification will be effective in the RFM registers only during the next read/transmit call.
 * This prevents to modify RFM status while in transmission/reception state.
 *
 * @param *handle    rfm95_handle_t structure containing RFM95 configurations and function pointers.
 * @param rfm95_cr_t cr             must be within 4/5 - 4/8 (rely on the dedicated rfm95_cr_t structure, do not manually write).
 *
 * @return void.
 */
void rfm95_set_CR(rfm95_handle_t *handle, rfm95_cr_t cr){
	handle->config.cr = cr;
	return;
}

/* Use this function to set RFM BW.
 * This function only updates the value in the handler.config structure.
 * The modification will be effective in the RFM registers only during the next read/transmit call.
 * This prevents to modify RFM status while in transmission/reception state.
 *
 * @param *handle    rfm95_handle_t structure containing RFM95 configurations and function pointers.
 * @param rfm95_bw_t bw             must be within 62.5 - 500 kHz (rely on the dedicated rfm95_cr_t structure, do not manually write).
 *
 * @return void.
 */
void rfm95_set_BW(rfm95_handle_t *handle, rfm95_bw_t bw){
	handle->config.bandwidth = bw;
	return;
}

/* Use this function to set RFM TX/RX channel frequency.
 * This function only updates the value in the handler.config structure.
 * The modification will be effective in the RFM registers only during the next read/transmit call.
 * This prevents to modify RFM status while in transmission/reception state.
 *
 * @param *handle  rfm95_handle_t structure containing RFM95 configurations and function pointers.
 * @param uint32_t freq           must be within 863 - 870 MHz.
 *
 * @return void.
 */
void rfm95_set_frequency(rfm95_handle_t *handle, uint32_t freq){
	uint32_t f = freq;
	if (f < LOW_FREQ_BAND_EU868)  f = LOW_FREQ_BAND_EU868;
	if (f > HIGH_FREQ_BAND_EU868) f = HIGH_FREQ_BAND_EU868;

	handle->config.channel_freq = f;
	return;
}

/* Use this function to set RFM Sync. word used for preamble detection.
 * This function only updates the value in the handler.config structure.
 * The modification will be effective in the RFM registers only during the next read/transmit call.
 * This prevents to modify RFM status while in transmission/reception state.
 *
 * @param *handle  rfm95_handle_t structure containing RFM95 configurations and function pointers.
 * @param uint8_t syncWord        any value (0x12 suggested, avoid 0x34 reserved for LoRaWAN).
 *
 * @return void.
 */
void rfm95_set_syncWord(rfm95_handle_t *handle, uint8_t syncWord){
	uint8_t sync = syncWord;

	if(sync == 0x34) sync = LORA_DEF_SYNC_WORD;
	handle->config.sync_word = sync;
	return;
}

/* This function allows to read the RSSI of the latest received packet.
 * IMPORTANT SNR register must be read before switching RFM95 to standby (i.e. immediately after receiving
 * a new packet).
 *
 * @param *handle  rfm95_handle_t   structure containing RFM95 configurations and function pointers.
 * @param *int16_t rssi             this variable will store the actual rssi value.
 *
 * @return true if no errors occurred.
 */
bool rfm95_getRSSI(rfm95_handle_t *handle, int16_t *rssi){
  uint8_t read;
  if(!read_register(handle, RFM95_REGISTER_PKT_RSSI, &read, 1)) return false;

  *rssi = -157 + read;
  return true;
}

/* This function allows to read the SNR of the latest received packet.
 * IMPORTANT RSSI register must be read before switching RFM95 to standby (i.e. immediately after receiving
 * a new packet).
 *
 * @param *handle  rfm95_handle_t   structure containing RFM95 configurations and function pointers.
 * @param *int16_t snr              this variable will store the actual snr value.
 *
 * @return true if no errors occurred.
 */
bool rfm95_getSNR(rfm95_handle_t *handle, int8_t *snr){
  uint8_t read;
  if(!read_register(handle, RFM95_REGISTER_PACKET_SNR, &read, 1)) return false;

  *snr =  (((int8_t)read)/4);
  return true;
}

/* Get live indication of LoRa modem status:
 * bit4 set -> modem clear
 * bit3 set -> Header info valid (valid CRC detected)
 * bit2 set -> RX on-going
 * bit1 set -> signal synchronized (end of preamble, modem in in lock)
 * bit0 set -> signal detected (valid LoRa preamble detected)
 *
 * The user is expected to manually scan those bits to get the current modem operating mode.
 *
 * @param *handle  rfm95_handle_t   structure containing RFM95 configurations and function pointers.
 * @param *uint8_t status           this variable will store only the 5 LSBs of the modem status register (1st 3 bits ignored).
 *
 * @return true if no errors occurred.
 */
bool rfm95_getModemStatus(rfm95_handle_t *handle, uint8_t *status){
  uint8_t read;
  if(!read_register(handle, RFM95_REGISTER_MODEM_STATUS, &read, 1)) return false;

  *status = read & 0x1F;
  return true;
}

/* This function sets rfm95 in standby mode.
 *
 * @param *handle rfm95_handle_t structure containing RFM95 configurations and function pointers
 *
 * @return true if no errors occurred
 */
bool rfm95_stdby(rfm95_handle_t *handle){
  // Move modem to LoRa standby
  if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_OP_MODE_LORA_STANDBY)) return false;

  // Wait for the modem to be ready
  if (!wait_for_irq(handle, RFM95_INTERRUPT_DIO5, RFM95_WAKEUP_TIMEOUT)) return false;

  handle->rfm_status = STNBY_MODE;

  return true;
}

/* This function allows to set RFM in TX mode and waits until the process is completed
 * or a timeout is generated.
 *
 * @param *handle  rfm95_handle_t   structure containing RFM95 configurations and function pointers.
 * @param *uint8_t senda_daya       pointer to data buffer to be transmitted.
 * @param size_t   send_data_length size for the TX buffer
 *
 * @return true if no errors occurred.
 */
bool rfm95_send(rfm95_handle_t *handle, const uint8_t *send_data, size_t send_data_length){

  // make changes effective
  if (!rfm95_modify_power(handle))     return false;
  if (!rfm95_modify_SF(handle))        return false;
  if (!rfm95_modify_frequency(handle)) return false;
  if (!rfm95_modify_CR_BW(handle))     return false;
  if (!rfm95_modify_syncWord(handle))  return false;

  // Set the payload length.
  if (!write_register(handle, RFM95_REGISTER_PAYLOAD_LENGTH, send_data_length)) return false;

  // Enable tx-done interrupt, clear flags and previous interrupt time
  if (!write_register(handle, RFM95_REGISTER_DIO_MAPPING_1, RFM95_DIO_MAPPING_1_IRQ_FOR_TXDONE)) return false;
  if (!write_register(handle, RFM95_REGISTER_IRQ_FLAGS, 0xFF)) return false;
  handle->interrupt_times[RFM95_INTERRUPT_DIO0] = 0;
  handle->interrupt_times[RFM95_INTERRUPT_DIO1] = 0;
  handle->interrupt_times[RFM95_INTERRUPT_DIO5] = 0;

  // Move modem to LoRa standby
  if (handle->rfm_status != STNBY_MODE){
    if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_OP_MODE_LORA_STANDBY)) return false;

    // Wait for the modem to be ready
    if (!wait_for_irq(handle, RFM95_INTERRUPT_DIO5, RFM95_WAKEUP_TIMEOUT)) return false;
    handle->rfm_status = STNBY_MODE;
  }

  // Set pointer to start of TX section in FIFO
  if (!write_register(handle, RFM95_REGISTER_FIFO_ADDR_PTR, RFM95_FIFO_TX_BASE_ADDRESS)) return false;

  // Write payload to FIFO.
  for (size_t i = 0; i < send_data_length; i++) {
    write_register(handle, RFM95_REGISTER_FIFO_ACCESS, send_data[i]);
  }

  // Set modem to tx mode.
  if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_OP_MODE_LORA_TX)) return false;

  // Wait for the modem to be ready
  if (!wait_for_irq(handle, RFM95_INTERRUPT_DIO5, RFM95_WAKEUP_TIMEOUT)) return false;
  handle->rfm_status = TRANSMIT_MODE;

  // Wait for the transfer complete interrupt.
  if (!wait_for_irq(handle, RFM95_INTERRUPT_DIO0, RFM95_SEND_TIMEOUT)) return false;

  // LSE Tick corresponding to the end of TX --> not needed here
  uint32_t tx_ticks = handle->interrupt_times[RFM95_INTERRUPT_DIO0];
  handle->interrupt_times[RFM95_INTERRUPT_DIO0] = 0;

  // Return modem to sleep.
  if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_OP_MODE_LORA_SLEEP)) return false;
  handle->rfm_status = SLEEP_MODE;

  return 1;
}

/* This function allows to set RFM in RX continuous mode. RX DONE interrupt on DIO0 is enabled, i.e.
 * the MCU can move to other tasks waiting for IRQ event.
 *
 * @param *handle  rfm95_handle_t   structure containing RFM95 configurations and function pointers.
 *
 * @return true if no errors occurred.
 */
bool rfm95_enter_rx_mode(rfm95_handle_t *handle){

  // Clear flags and previous interrupt time, configure mapping for RX done.
  if (!write_register(handle, RFM95_REGISTER_DIO_MAPPING_1, RFM95_DIO_MAPPING_1_IRQ_FOR_RXDONE)) return false;
  if (!write_register(handle, RFM95_REGISTER_IRQ_FLAGS, 0xFF)) return false;
  handle->interrupt_times[RFM95_INTERRUPT_DIO0] = 0;
  handle->interrupt_times[RFM95_INTERRUPT_DIO1] = 0;
  handle->interrupt_times[RFM95_INTERRUPT_DIO5] = 0;

  // Move modem to LoRa standby
  if (handle->rfm_status != STNBY_MODE){
    if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_OP_MODE_LORA_STANDBY)) return false;

    // Wait for the modem to be ready.
    if (!wait_for_irq(handle, RFM95_INTERRUPT_DIO5, RFM95_WAKEUP_TIMEOUT)) return false;
    handle->rfm_status = STNBY_MODE;
  }

  // Enter RX CONT mode
  if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_OP_MODE_LORA_RX_CONT)) return false;

  // Wait for the modem to be ready.
  handle->interrupt_times[RFM95_INTERRUPT_DIO5] = 0;
  if (!wait_for_irq(handle, RFM95_INTERRUPT_DIO5, RFM95_WAKEUP_TIMEOUT)) return false;
  handle->rfm_status = RXCONTIN_MODE;

	return 1;
}

/* This function allows reading the latest received packet from the RFM internal FIFO.
 *
 * @param *handle  rfm95_handle_t   structure containing RFM95 configurations and function pointers.
 * @param *uint8_t rx_buff          will contain the received payload.
 * @param size_t   rx_data_length   will contain the received number of bytes.
 *
 * @return true if no errors occurred.
 */
bool rfm95_receive(rfm95_handle_t *handle, uint8_t *rx_buff, uint8_t *rx_data_length){

  // Move modem to LoRa standby.
  if (handle->rfm_status != STNBY_MODE){
    if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_OP_MODE_LORA_STANDBY)) return false;

    // Wait for the modem to be ready.
    if (!wait_for_irq(handle, RFM95_INTERRUPT_DIO5, RFM95_WAKEUP_TIMEOUT)) return false;
    handle->rfm_status = STNBY_MODE;
  }

  // proceed with payload extraction:
  uint8_t irq_flags;
  read_register(handle, RFM95_REGISTER_IRQ_FLAGS, &irq_flags, 1);

  // Check if there was a CRC error.
  if (irq_flags & RFM95_PAYLOAD_CRC_ERR_MSK) {
    // Return modem to sleep.
    if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_OP_MODE_LORA_SLEEP)) return false;
    handle->rfm_status = SLEEP_MODE;
    return false;
  }

  // Read received payload length.
  uint8_t rx_bytes;
  if (!read_register(handle, RFM95_REGISTER_FIFO_RX_BYTES_NB, &rx_bytes, 1)) return false;

  // block here if no bytes were received
  if (rx_bytes == 0) {
    write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_OP_MODE_LORA_SLEEP);
    return false;
  }

  // Read packet location within the FIFO buffer
  uint8_t fifo_rx_entry;
  if (!read_register(handle, RFM95_REGISTER_FIFO_RX_CURR_ADDR, &fifo_rx_entry, 1)) return false;

  // Read received payload itself.
  if (!write_register(handle, RFM95_REGISTER_FIFO_ADDR_PTR, fifo_rx_entry)) return false;
  if (!read_register(handle, RFM95_REGISTER_FIFO_ACCESS, rx_buff, rx_bytes))return false;

  // Return modem to sleep --> needed to clear the FIFO
  if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_OP_MODE_LORA_SLEEP)) return false;
  handle->rfm_status = SLEEP_MODE;

  *rx_data_length = rx_bytes;

	return true;
}


/* This function is called by EXTI interrupt events. The corresponding RFM95
 * event is registered into this dedicated buffer with very precise timing.
 *
 * @param *handle   rfm95_handle_t structure containing RFM95 configurations and function pointers.
 * @param interrupt rfm95_interrupt_t defining which interrupt triggered the IRQ (either DIO0, DIO1 or DIO5).
 *
 * @return void.
 */
void rfm95_on_interrupt(rfm95_handle_t *handle, rfm95_interrupt_t interrupt) {
  if (handle->rfm_timer->Instance->CR & LPTIM_CR_ENABLE) {
    // get timing if and only if LPTIM is running
    handle->interrupt_times[interrupt] = handle->get_precision_tick();
    return;
  }

  // else, skip...
  handle->interrupt_times[interrupt] = 0;
}

