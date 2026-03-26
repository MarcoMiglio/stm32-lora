/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "lora.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// rfm95 handler
rfm95_handle_t rfm95_handle = {0};

// RX PKT
typedef struct{
  uint8_t tx_attempts;  // TX attempts performed for this specific payload

  bool ack;             // Track ACKs for this packet

  uint8_t nodeID;       // Node ID for this packet

  uint16_t pktID;       // pkt ID to identify this specific payload

  uint8_t rx_bcID;      // ID of the breadcrumb that has received this packet from the environmental node

  uint8_t tx_bcID;      // ID of the breadcrumb that has transmitted this packet to this node

  uint8_t pl_len;       // Number of bytes of the entire packet

  uint8_t pl[LORA_PAYLOAD_MAX_SIZE]; // Pointer to the entire payload received (Buffer size fixed to avoid dynamic memory allocation)
} bc_pkt;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
LPTIM_HandleTypeDef hlptim1;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
// ------------- LPTIM ----------------
volatile uint32_t lptim_tick_msb = 1;

// handle RX and TX events
bool rx_data;
bool tx_data;

volatile bool dmaRunning = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI3_Init(void);
static void MX_LPTIM1_Init(void);
static void MX_RNG_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
int _write(int file, char *ptr, int len);
void debug_pin_set();
void debug_pin_rst();

static bool     init_rfm();
static uint32_t get_precision_tick();
static void 		precision_sleep_until(uint32_t target_ticks);
static uint8_t	get_battery_level();
static void     rfm95_after_interrupts_configured();

events_flags onRxEvt(rfm95_handle_t *h_rfm, bc_pkt *rx_pkt);
events_flags onTxEvt(rfm95_handle_t *h_rfm, bc_pkt *tx_pkt);

uint8_t get_random_number(uint32_t *random_number, uint16_t timeout);
static uint16_t random_wait(uint16_t min, uint16_t max);
static uint32_t get_currTime_ms();
static inline int julian_day(int year, int month, int day);

void MySystemClock_Config(void);
void enterStopMode();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_RTC_Init();
  MX_SPI3_Init();
  MX_LPTIM1_Init();
  MX_RNG_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // needed after programming -> avoid conflicts with sleep mode
  HAL_Delay(2000);

  // init RF in RX mode
  if (!init_rfm()) printf("Error during RFM initialization\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){

    bc_pkt rx_pkt = {0};

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (rx_data) {

      // clear RX flag
      rx_data = 0;

      // Something recevived -> Process informations
      events_flags rx_flags = {0};
      rx_flags = onRxEvt(&rfm95_handle, &rx_pkt);

      if (rx_flags.err_flags != 0) {
        // Error occurred...
        bool spi_err = rx_flags.err_flags & EVT_RFM_SPI_ERR;
        bool rx_err  = rx_flags.err_flags & EVT_RFM_RX_ERR;

        if (spi_err) {
          printf("SPI ERROR\r\n");

          // reset RFM and restart in RX mode
          HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
          reset_rfm(&rfm95_handle);
          init_rfm();
        }
        if (rx_err) {
          // PKT dropped, do nothing...
        }

      } else {
        // Plot payload on serial:
        HAL_UART_Transmit_DMA(&huart1, rx_pkt.pl, rx_pkt.pl_len);
        dmaRunning = true;

        // back in RX mode
        if (!tx_data) rfm95_enter_rx_mode(&rfm95_handle);
        else {
          // Tx pending -> before going back to RX mode TX must be performed!
        }
      }

    }

    if (tx_data) {  /* TX Pending */

      // TX ACK for the BC nodes
      events_flags tx_err_flags = {0};
      tx_err_flags = onTxEvt(&rfm95_handle, &rx_pkt);

      if (tx_err_flags.err_flags != 0) {
        // Error occurred...
        bool spi_err = tx_err_flags.err_flags & EVT_RFM_SPI_ERR;

        if (spi_err) {
          printf("SPI ERROR\r\n");

          // reset RFM and restart in RX mode
          HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
          reset_rfm(&rfm95_handle);
          init_rfm();
        }
      } else {
        // back in RX mode
        rfm95_enter_rx_mode(&rfm95_handle);
      }

      tx_data = 0;  // clear TX flag
    }

  } // End of While superloop
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV8;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  // Init RTC with reference date
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = START_MONTH;
  sDate.Date = START_DAY;
  sDate.Year = START_YEAR;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  // make sure RTC wkup is not running:
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */
  /*
   * This dummy sequence sets clock LOW when idle, MOSI and MISO to a definite state (either HIGH or LOW)
   */
   uint8_t dummy = 0xFF;
   HAL_SPI_Transmit(&hspi3, &dummy, 1, 10);

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RFM95_RST_GPIO_Port, RFM95_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RFM95_CS_GPIO_Port, RFM95_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Test_GPIO_Port, Test_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC0 PC1 PC2
                           PC3 PC4 PC5 PC6
                           PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PH0 PH1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 PA7
                           PA8 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB12 PB13 PB14
                           PB15 PB4 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RFM95_DIO0_Pin RFM95_DIO1_Pin RFM95_DIO5_Pin */
  GPIO_InitStruct.Pin = RFM95_DIO0_Pin|RFM95_DIO1_Pin|RFM95_DIO5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RFM95_RST_Pin */
  GPIO_InitStruct.Pin = RFM95_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RFM95_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RFM95_CS_Pin */
  GPIO_InitStruct.Pin = RFM95_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(RFM95_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Test_Pin */
  GPIO_InitStruct.Pin = Test_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Test_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

//  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// -------------- RFM95 user defined functions -------------------

/*
 *
 */
bool init_rfm(){

  // Start lptim timer
  HAL_LPTIM_Counter_Start_IT(&hlptim1, 0xFFFF);

  // config RFM95
  rfm95_handle.rfm_timer  = &hlptim1;
  rfm95_handle.spi_handle = &hspi3;
  rfm95_handle.nrst_port  = RFM95_RST_GPIO_Port;
  rfm95_handle.nrst_pin   = RFM95_RST_Pin;
  rfm95_handle.nss_port   = RFM95_CS_GPIO_Port;
  rfm95_handle.nss_pin    = RFM95_CS_Pin;

  rfm95_handle.precision_tick_frequency = LSE_CLK;
  rfm95_handle.precision_tick_drift_ns_per_s = 20000;
  rfm95_handle.get_precision_tick = get_precision_tick;
  rfm95_handle.precision_sleep_until = precision_sleep_until;
  rfm95_handle.on_after_interrupts_configured = rfm95_after_interrupts_configured;
  //rfm95_handle.random_int = random_int;
  rfm95_handle.get_battery_level = get_battery_level;

  // Modify parameters here:
  rfm95_set_power(&rfm95_handle, LORA_TX_POWER); // power 2 dBm - 17 dBm
  rfm95_set_frequency(&rfm95_handle, LORA_CH_FREQ);
  rfm95_set_BW(&rfm95_handle, LORA_BW);
  rfm95_set_CR(&rfm95_handle, LORA_CR);
  rfm95_set_SF(&rfm95_handle, LORA_SF);
  rfm95_set_syncWord(&rfm95_handle, LORA_SYNC_WORD);

  // initialize RFM95
  if(!rfm95_init(&rfm95_handle)) return false;

  // set RFM95 to continuous RX mode
  if(!rfm95_enter_rx_mode(&rfm95_handle)) return false;

  return true;
}

/*
 * return precise timing based on internal LPTIM module
 */
static uint32_t get_precision_tick(){
	__disable_irq();
	uint32_t precision_tick = lptim_tick_msb | HAL_LPTIM_ReadCounter(&hlptim1);
	__enable_irq();
	return precision_tick;
}

/*
 * Allows to set the MCU into STOP2 mode with accurate wkup timer
 */
static void precision_sleep_until(uint32_t target_ticks){
	while(1){
		uint32_t curr_tick = get_precision_tick();

		// exit condition:
		if(target_ticks < curr_tick) break;

		uint32_t sleep_ticks = target_ticks - curr_tick;

		// avoid short sleep intervals:
		if(sleep_ticks < 10) break;
		else {
			// overall CMP value - some margin (needed to reset clock configurations after stop mode2)
			uint32_t compare_tick = (curr_tick & 0xFFFF) + sleep_ticks - 10;

			if (compare_tick >= 0xFFFF){ // ARR will awake MCU before compare
				HAL_SuspendTick();
				enterStopMode();
			} else { // otherwise CMP keeps MCU sleep all time
				__HAL_LPTIM_CLEAR_FLAG(&hlptim1, LPTIM_FLAG_CMPOK);
				__HAL_LPTIM_COMPARE_SET(&hlptim1, compare_tick);            // set CMP limit

				while (!__HAL_LPTIM_GET_FLAG(&hlptim1, LPTIM_FLAG_CMPOK));  // wait for effective change

				__HAL_LPTIM_CLEAR_FLAG(&hlptim1, LPTIM_FLAG_CMPM);          // clear CMP interrupt flag
				__HAL_LPTIM_ENABLE_IT(&hlptim1, LPTIM_IT_CMPM);             // enable CMP interrupt

				HAL_SuspendTick();
				enterStopMode();

				__HAL_LPTIM_DISABLE_IT(&hlptim1, LPTIM_IT_CMPM);            // disable CMP interrupt
			}
		}
	}

	while(get_precision_tick() < target_ticks);                       // wait residue time here ( < 10 ticks)
}


static uint8_t get_battery_level(){
	return 0;
}

/*
 * This function is executed after initializing rfm95 (ready to accept interrupts
 * without hard fault errors)
 */
void rfm95_after_interrupts_configured(){
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* Get random number from internal RNG.
 * This function automatically manages activation/de-activation of RNG module to save power
 *
 * return values:
 * - 0 = no errors
 * - 1 = timeout
 * - 2 = seed error  -> RNG must be reinitialized (random number must be discarded)
 * - 3 = Clock error -> ensure rng_clk >= ahb_clk/16 (>= 5 MHz)
 */
uint8_t get_random_number(uint32_t *random_number, uint16_t timeout){

  // enable RNG peripheral:
  __HAL_RNG_ENABLE(&hrng);

  // clear clock error and seed error interrupt flags:
  __HAL_RNG_CLEAR_IT(&hrng, RNG_IT_CEI);
  __HAL_RNG_CLEAR_IT(&hrng, RNG_IT_SEI);

  // wait for data ready bit to be set:
  uint32_t start = HAL_GetTick();
  while(!__HAL_RNG_GET_FLAG(&hrng, RNG_FLAG_DRDY)){
    if (HAL_GetTick() - start > timeout) {
      __HAL_RNG_DISABLE(&hrng);
      return 1;
    }
  }

  // RNG can be switched off here:
  __HAL_RNG_DISABLE(&hrng);

  // check seed error:
  if(__HAL_RNG_GET_IT(&hrng, RNG_IT_SEI)) return 2;

  // check clock error:
  if(__HAL_RNG_GET_IT(&hrng, RNG_IT_CEI)) return 3;

  *random_number = hrng.Instance->DR;

  // check event seed error occurred while loading data:
  if(*random_number == 0) return 2;

  return 0;
}

/* Generate random number in range [min, max] expressed in
 * milliseconds
 *
 * If min > max, or max == 0, the random wait is skipped,
 * and 0 is returned as default.
 *
 * @param uint16_t  max represents maximum time (in ms)
 * @param uint16_t  min represents minimum time (in ms)
 *
 * @return uint16_t in range [min, max]
 */
static uint16_t random_wait(uint16_t min, uint16_t max){
  if ((min >= max) || (max == 0)) return 0;

  uint32_t num;
  uint8_t rng_err = get_random_number(&num, 1);

  if (rng_err){
    // TODO handle potential rng seed error/clock error here...

    num = 0;
  }

  uint16_t range = max - min + 1;
  return (uint16_t)(min + (num % range));
}

/*
 * Julian day algorithm... Not really needed here
 */
static inline int julian_day(int year, int month, int day){
  if (month <= 2) {
    year -= 1;
    month += 12;
  }
  int A = year / 100;
  int B = 2 - A + (A / 4);
  int jd = (int)(365.25 * (year + 4716))
         + (int)(30.6001 * (month + 1))
         + day + B - 1524;
  return jd;
}

/*
 * Get time elapsed from program start (in ms)
 *
 * @return uint32_t expressing time elapsed since program start (in ms)
 */
static uint32_t get_currTime_ms(){
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);  // latch time

  // compute time elapsed in days
  int ref_day = julian_day(START_YEAR_DEC, START_MONTH_DEC, START_DAY_DEC);
  int today   = julian_day(sDate.Year, sDate.Month, sDate.Date);

  uint32_t days_elapsed = today - ref_day;

  uint32_t days_elapsed_ms = days_elapsed * MS_PER_DAY;

  // read milliseconds register:
  uint16_t milliseconds = (uint16_t)(((sTime.SecondFraction - sTime.SubSeconds) * 1000U) / (sTime.SecondFraction + 1));

  // milliseconds elapsed since reference date:
  uint32_t now_ms = days_elapsed_ms + ((sTime.Hours * S_PER_HOUR + sTime.Minutes * S_PER_MIN + sTime.Seconds)*MS_PER_S) + milliseconds;

  return now_ms;
}

/*
 * On RX the HEAD node will behave as following:
 *
 * - discard invalid pkts (those with wrong formatting)
 * - TX full (unformatted) payload on serial port
 * - determine if a TX is needed as an ACK:
 *      TX are always performed except for:
 *            * PKTs received directly from environmental nodes
 *            * ALARM ACK PKTs, i.e. a node specifically scheduling a TX to send an ACK
 *              to a node behind in the sequence ( in this case, the node which is transmistting
 *              the ACK has already received its ACK, so it dosen't need an additional TX from us!)
 *
 *
 * @param rfm95_handle_t *h_rfm pointer to RFM handler
 * @param bc_pkt *rx_pkt pointer to structure holding all the informations for PKTs travelling
 *                       in the BCs sequence.
 *
 * @return event_flags falgs for RX events (see app_events.h in other files...)
 */
events_flags onRxEvt(rfm95_handle_t *h_rfm, bc_pkt *rx_pkt) {
  /* track error flags */
  events_flags app_flags = {0};
  tx_data = false;

  /* set standby mode to read data from rfm95 */
  if(!rfm95_stdby(h_rfm)) app_flags.err_flags |= EVT_RFM_SPI_ERR;

  /* read received data */
  if(!rfm95_receive(h_rfm, &rx_pkt->pl[0], &rx_pkt->pl_len))app_flags.err_flags |= EVT_RFM_RX_ERR;

  /* If any error occurred, stop code here */
  if(app_flags.err_flags != 0) {
    return app_flags;
  }

  /* read received sync word */
  uint16_t rx_sync = (uint16_t)((rx_pkt->pl[SYNC_WORD_POS] << 8) | rx_pkt->pl[SYNC_WORD_POS+1]);

  /* preliminary check on payload size and SYNC words */
  if (!( ((rx_pkt->pl_len == ENV_NODE_PYL_SIZE) && (rx_sync == SYNC_WORD_ENV)) ||
         ((rx_pkt->pl_len >= BC_NODE_MIN_PYL_SIZE) && (rx_sync == SYNC_WORD_BC)) )) {

    app_flags.err_flags |= EVT_BAD_PKT_FORMAT;
    return app_flags;
  }

  // extract nodeID
  rx_pkt->nodeID = rx_pkt->pl[NODE_ID_POS];

  // byte1 and byte2 = pktID
  rx_pkt->pktID = (rx_pkt->pl[PKT_ID_MSB_POS] << 8) | rx_pkt->pl[PKT_ID_LSB_POS];

  // get entire mask field
  uint8_t pkt_mask = rx_pkt->pl[MASK_POS];

  // get alarm bit
  bool alarm_bit_set = (pkt_mask & MASK_ALARM_BIT) >> ALARM_BIT_POS;

  // rx_bcID and tx_bcID depend on subsequent conditions
  if ((rx_pkt->pl_len == ENV_NODE_PYL_SIZE) && (rx_sync == SYNC_WORD_ENV)) {       /* receiving from an ENV NODE */

    // simply plot ENV node data, no TX needed...

  } else if ((rx_pkt->pl_len > ENV_NODE_PYL_SIZE) && (rx_sync == SYNC_WORD_BC)) {  /* receiving fron BC NODE -> some hops happened */

    /*
     * if receiving from bcNode, at leat one Hop happened
     * -> extarct 1st bc ID and last bc ID in the hopping sequence
     */
    rx_pkt->tx_bcID = rx_pkt->pl[rx_pkt->pl_len - 1];  // last RX byte corresponds to the BC_ID of the last BC in the hop-sequence

    // get the BC_ID of the 1st BC in the hop-sequence
    rx_pkt->rx_bcID = rx_pkt->pl[BC_ID1_POS];

    if (rx_pkt->tx_bcID > MY_BC_ID) {    /* RX from node farther in the BCs sequence --> UPLINK */

      if (alarm_bit_set){ /* Uplink for ALARM PKTs */

        bool is_ack = ((rx_pkt->pl[MASK_POS] & MASK_ALARM_ACK) >> ACK_BIT_POS) & 0x01;
        // If set no TX needed...

        // If not set -> ACK is needed
        if(!is_ack) tx_data = true;

      } else {            /* Handle Uplink for normal PKTs */

        // respond back with MY BC ID added
        tx_data = true;

      }


    } else {                            /* RX from node ahead in the BCs sequence --> ACK */

      // Error... This node is the head!

    }

  } else {                              /* BAD PKT format */

    app_flags.err_flags |= EVT_BAD_PKT_FORMAT;

  }

  return app_flags;
}

/*
 * On TX event HEAD node simply retransmitts the entire payload by adding
 * its own informations (i.e. BC_ID = 0).
 *
 * Indeed, other nodes in the sequence will treat the head BC-ID PKT as an ACK.
 * (Node with higher priority in teh sequence has received the intended pkt).
 */
events_flags onTxEvt(rfm95_handle_t *h_rfm, bc_pkt *tx_pkt){

  events_flags app_flags = {0};

  // Add This BC-informations before TX
  tx_pkt->pl[tx_pkt->pl_len] = MY_BC_ID;
  tx_pkt->pl_len = tx_pkt->pl_len + 1;

  /* TX payload here */
  if (!rfm95_send(h_rfm, tx_pkt->pl, tx_pkt->pl_len)) app_flags.err_flags |= EVT_RFM_SPI_ERR;
  if(app_flags.err_flags != 0) return app_flags;

  /* Set RFM back to RX mode */
  if (!rfm95_enter_rx_mode(h_rfm)) app_flags.err_flags |= EVT_RFM_SPI_ERR;
  if(app_flags.err_flags != 0) return app_flags;

  return app_flags;
}

// --------------------------------------------------------------------------------------



// -------------------------- System Power down routines --------------------------------

/*
 * Enter stop mode and resume clock configurations on exit
 */
void enterStopMode(){

  // Stop LPTIM and clear pending bits
  HAL_LPTIM_Counter_Stop_IT(&hlptim1);
  LPTIM1->ICR |= 0x3F;

  // CLear LPTIM IRQ at NVIC level
  NVIC_ClearPendingIRQ(LPTIM1_IRQn);

  // wake from HSI --> faster wake up sequence:
	__HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI);

	// Enter stop mode 2:
	HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);

	// awake here...

	// resume system clock:
	if(READ_BIT(RCC->CR, RCC_CR_PLLRDY) == 0U) MySystemClock_Config();

	// resume system tick with correct clock
	__enable_irq();
	HAL_ResumeTick();

	// Enable LPTIM again:
	HAL_LPTIM_Counter_Start_IT(&hlptim1, 0xFFFF);

	//while(get_precision_tick() == 0);
}


/*
 * Modified clock setup function:
 * -> Avoid repeating initialization for the LSE 32 kHz clock if already running.
 *    Doing this every time the MCU exits STOP mode, results in a corrupted timing accuracy.
 */
void MySystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability only if it
  */
  if ((HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSEON) == 0) || (HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSERDY)) != 1){
  	HAL_PWR_EnableBkUpAccess();
		__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

		RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
  }


  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType |= RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

// --------------------------------------------------------------------------------------



// ------------------------------------- Call-backs --------------------------------------

// Auto-reload callback for LPTIM module
void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim){
  lptim_tick_msb += 0x10000;
}

// Compare match callback for LPTIM module
void HAL_LPTIM_CompareMatchCallback(LPTIM_HandleTypeDef *hlptim){
  // do nothing...
}

// RTC wkup timer -> needed to schedule tx events
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc){

}

// GPIO external interrupts callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	// Events on RFM95 interrupt pins
  if (GPIO_Pin == RFM95_DIO0_Pin) {
    rfm95_on_interrupt(&rfm95_handle, RFM95_INTERRUPT_DIO0);

    // something received
    if(rfm95_handle.rfm_status == RXCONTIN_MODE) rx_data = true;

  } else if (GPIO_Pin == RFM95_DIO1_Pin) {
    rfm95_on_interrupt(&rfm95_handle, RFM95_INTERRUPT_DIO1);
  } else if (GPIO_Pin == RFM95_DIO5_Pin) {
    rfm95_on_interrupt(&rfm95_handle, RFM95_INTERRUPT_DIO5);
  }

}

// USART callback
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    dmaRunning = false;
  }
}

// ----------------------------------------------------------------------------------------



// --------------------------------- DEBUG FUNCTIONS --------------------------------------

/*
 * Function used to print on UART serial (DEBUG)
 */
int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}

/*
 * Use these two functions to SET/RESET the debug pin.
 * This allows to measure precise timing with an external DSO/Meter.
 *
 * example:
 *
 *  debug_pin_set();
 *  target_function();
 *  debug_pin_reset();
 *
 * allows to measure time execution of the target_function().
 */
void debug_pin_set(){
  HAL_GPIO_WritePin(Test_GPIO_Port, Test_Pin, GPIO_PIN_SET);
}

void debug_pin_rst(){
  HAL_GPIO_WritePin(Test_GPIO_Port, Test_Pin, GPIO_PIN_RESET);
}

// -----------------------------------------------------------------------------------------

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
