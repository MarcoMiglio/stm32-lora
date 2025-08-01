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
#include "buff_manager.h"
#include "app_events.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// rfm95 handler
rfm95_handle_t rfm95_handle = {0};

// setup linked list to hanlde TX sequence
LL_node tx_buff[BUFF_FIFO_SIZE];
LL_handler h_tx = {
    .head = 0,
    .tail = 0,
    .ins_idx = 0,

    .ll_status = LL_EMPTY,

    .ll_buff = &tx_buff[0]
};

// define RX buffer for BC payloads
rnode h_rx[BUFF_FIFO_SIZE];

// setup handler to manage RX - TX events
h_rx_tx h_buffs = {
    .h_rx = &h_rx[0],
    .h_tx = &h_tx
};

// system handler
Sys_handler h_sys = {
    .evt_flags = SYS_EVT_NO_PENDING,
    .state     = SYS_RDY
};

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

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// ------------- LPTIM ----------------
volatile uint32_t lptim_tick_msb = 0;
uint32_t tx_evt_time = 0;

// ----------- RX-TX FIFO -------------
events_flags app_flags = {0};           // handle status flags from RX TX buffers operation

// --------- Variables used for simulation ---------
volatile bool tx_evt = false;

uint8_t tx_seq_idx = 0;

// --------- Build some random payloads ------------
uint8_t pyl1[SENSOR_PLD_BYTES] = {0xAA,0xAA,0xBB,0xBB};
uint8_t pyl1_len = 4;

uint8_t pyl2[SENSOR_PLD_BYTES] = {0xCC,0xDD};
uint8_t pyl2_len = 2;

uint8_t pyl3[SENSOR_PLD_BYTES] = {0xEE,0xFF,0xDD,0xFF,0xEE,0x00,0x00,0xFF};
uint8_t pyl3_len = 8;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI3_Init(void);
static void MX_LPTIM1_Init(void);
static void MX_RNG_Init(void);
/* USER CODE BEGIN PFP */
int _write(int file, char *ptr, int len);
void debug_pin_set();
void debug_pin_rst();

static bool     init_rfm();
static uint32_t get_precision_tick();
static void 		precision_sleep_until(uint32_t target_ticks);
static uint8_t	get_battery_level();
static void     rfm95_after_interrupts_configured();

uint8_t get_random_number(uint32_t *random_number, uint16_t timeout);
static uint16_t random_wait(uint16_t min, uint16_t max);
static void schedule_tx_evt(uint16_t min, uint16_t max);

void MySystemClock_Config(void);
void enterStopMode();

void print_LL();
void build_pkt(uint16_t sync, uint8_t mask, uint8_t nodeID, uint16_t pktID, uint8_t* sensors, uint8_t sens_len,
		           int16_t rssi, uint8_t* bc_seq, uint8_t bc_seq_len, uint8_t* t_buff, uint8_t* t_buff_len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// -------------- Define here the test sequence ----------------

// ----> ENV NODE 1 (define here PKTs from operator 1)
test_buff e1_p1_bc22  = {0};
test_buff e1_p1_bc54  = {0};
test_buff e1_p1_bc543 = {0};
test_buff e1_p1_bc51  = {0};
test_buff e1_p1_bc64  = {0};
test_buff e1_p2_bc41  = {0};
test_buff e1_p1_bc21  = {0};
test_buff e1_p1_bc31  = {0};
test_buff e1_p2_bc22  = {0};
test_buff e1_p2_bc43  = {0};

// ----> ENV NODE 2 (define here PKTs from operator 2)
test_buff e2_p1_bc51  = {0};

// ----> ENV NODE 3 (define here PKTs from operator 3)
test_buff e3_p1_bc53  = {0};
test_buff e3_p1_bc64  = {0};


// ----> GENERAL BUFF (This one defines the test TX sequence)
test_buff* test_buffs[] = {
		&e1_p1_bc543,
		&e1_p1_bc51,
		&e1_p1_bc54,
		&e1_p2_bc43,
//		&e1_p1_bc64,
//		&e1_p1_bc22,
		&e1_p1_bc22,
		&e1_p2_bc22,
		&e1_p1_bc64,
		&e1_p1_bc22,
//		&e1_p1_bc21,
//		&e1_p2_bc41,
//		&e1_p1_bc31,
//		&e2_p1_bc51,
		&e3_p1_bc53,
		&e3_p1_bc64
};

#define TEST_SEQ_SIZE (sizeof(test_buffs)/sizeof(test_buffs[0]))

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

	// ---- POPOLATE TEST BUFFER ----
  uint8_t bc_seq[20];

  /*
   * VARIABLES FOR BUILDING PKT ARRAY:
   *
   * uint8_t mask, uint8_t nodeID, uint16_t pktID, uint8_t* sensors, uint8_t sens_len,
   * int16_t RSSI, uint8_t* bc_seq, uint8_t bc_seq_len, uint8_t* t_buff, uint8_t* t_buff_le
   */

	// ----> ENV NODE 1
	build_pkt(SYNC_WORD_ENV, 0, 1, 1, pyl1, pyl1_len, 0, bc_seq, 0, e1_p1_bc22.buff, &e1_p1_bc22.len);

	build_pkt(SYNC_WORD_ENV, 0, 1, 2, pyl2, pyl2_len, 0, bc_seq, 0, e1_p2_bc22.buff, &e1_p2_bc22.len);

	bc_seq[0] = 5;
	bc_seq[1] = 4;
	build_pkt(SYNC_WORD_BC, 0, 1, 1, pyl1, pyl1_len, -89, bc_seq, 2, e1_p1_bc54.buff, &e1_p1_bc54.len);

	bc_seq[0] = 5;
	bc_seq[1] = 4;
	bc_seq[2] = 2;
	bc_seq[3] = 1;
	build_pkt(SYNC_WORD_BC, 0, 1, 1, pyl1, pyl1_len, -89, bc_seq, 4, e1_p1_bc51.buff, &e1_p1_bc51.len);

	bc_seq[0] = 5;
	bc_seq[1] = 4;
	bc_seq[2] = 3;
	build_pkt(SYNC_WORD_BC, 0, 1, 1, pyl1, pyl1_len, -92, bc_seq, 3, e1_p1_bc543.buff, &e1_p1_bc543.len);

	bc_seq[0] = 6;
	bc_seq[1] = 5;
	bc_seq[2] = 4;
	build_pkt(SYNC_WORD_BC, 0, 1, 1, pyl1, pyl1_len, -99, bc_seq, 3, e1_p1_bc64.buff, &e1_p1_bc64.len);

	bc_seq[0] = 2;
	bc_seq[1] = 1;
	build_pkt(SYNC_WORD_BC, 0, 1, 1, pyl1, pyl1_len, -123, bc_seq, 2, e1_p1_bc21.buff, &e1_p1_bc21.len);

	bc_seq[0] = 3;
	bc_seq[1] = 2;
	bc_seq[2] = 1;
	build_pkt(SYNC_WORD_BC, 0, 1, 1, pyl1, pyl1_len, -123, bc_seq, 3, e1_p1_bc31.buff, &e1_p1_bc31.len);

	bc_seq[0] = 4;
	bc_seq[1] = 3;
	bc_seq[2] = 1;
	build_pkt(SYNC_WORD_BC, 0, 1, 2, pyl3, pyl3_len, -74, bc_seq, 3, e1_p2_bc41.buff, &e1_p2_bc41.len);

	bc_seq[0] = 4;
	bc_seq[1] = 3;
	build_pkt(SYNC_WORD_BC, 0, 1, 2, pyl3, pyl3_len, -78, bc_seq, 2, e1_p2_bc43.buff, &e1_p2_bc43.len);


	// ----> ENV NODE 2
	bc_seq[0] = 5;
	bc_seq[1] = 3;
	bc_seq[2] = 1;
	build_pkt(SYNC_WORD_BC, 0, 2, 1, pyl2, pyl2_len,  -80, bc_seq, 3, e2_p1_bc51.buff, &e2_p1_bc51.len);


	// ----> ENV NODE 3
	bc_seq[0] = 5;
	bc_seq[1] = 4;
	bc_seq[2] = 3;
	build_pkt(SYNC_WORD_BC, 0, 3, 1, pyl3, pyl3_len,  -91, bc_seq, 3, e3_p1_bc53.buff, &e3_p1_bc53.len);

	bc_seq[0] = 6;
	bc_seq[1] = 5;
	bc_seq[2] = 4;
	build_pkt(SYNC_WORD_BC, 0, 3, 1, pyl3, pyl3_len, -101, bc_seq, 3, e3_p1_bc64.buff, &e3_p1_bc64.len);


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
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_SPI3_Init();
  MX_LPTIM1_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */

  printf("start\r\n");

  // init RX and TX handler
  init_buffers(&h_buffs);

  // init RF in RX mode
  if (!init_rfm()) printf("Error during RFM initialization\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  	if (tx_evt == true) {
  		tx_evt = false;

  		uint8_t c_len  = test_buffs[tx_seq_idx]->len;
  		uint8_t c_node = test_buffs[tx_seq_idx]->buff[NODE_ID_POS];
  		uint16_t c_pkt = ((uint16_t)(test_buffs[tx_seq_idx]->buff[PKT_ID_MSB_POS] << 8)) | test_buffs[tx_seq_idx]->buff[PKT_ID_LSB_POS];
  		uint8_t  c_bc1 = test_buffs[tx_seq_idx]->buff[BC_ID1_POS];
  		uint8_t  c_bcL = test_buffs[tx_seq_idx]->buff[c_len-1];

  		printf("\n###### TX PKT E%d-P%d-%d%d\r\n",c_node,c_pkt,c_bc1,c_bcL);

  		/* Prepare TX buff here */

  		/* TX payload here */
			if (!rfm95_send(&rfm95_handle, test_buffs[tx_seq_idx]->buff, test_buffs[tx_seq_idx]->len)) printf("TX error");
			tx_seq_idx+=1;

			if (tx_seq_idx >= TEST_SEQ_SIZE) tx_seq_idx = 0;

  		/* set RFM95 back to continuous RX mode */
			if(!rfm95_enter_rx_mode(&rfm95_handle)) printf("RFM err entering RX\r\n");
  	}

  	if (h_sys.evt_flags & SYS_EVT_RX_PENDING) {
  		 h_sys.evt_flags &= ~SYS_EVT_RX_PENDING;

  		 // read rfm data and print
  		 uint8_t rx_buff[LORA_PAYLOAD_MAX_SIZE];
  		 uint8_t rx_buff_len;

  		 /* set standby mode to read data from rfm95 */
			 if(!rfm95_stdby(&rfm95_handle)) printf("RFM err stdby\r\n");

			 /* read received data */
			 if(!rfm95_receive(&rfm95_handle, &rx_buff[0], &rx_buff_len)) {

				 printf("RX error");
				 // do domething...

			 } else {

				 if (rx_buff_len < ENV_NODE_PYL_SIZE) {

					 printf("\nPKT too short, drop\r\n");

				 } else {

					 uint16_t sync = (uint16_t)((rx_buff[SYNC_WORD_POS] << 8) | (rx_buff[SYNC_WORD_POS+1]));

					 if (sync != SYNC_WORD_BC) { /* Check if TX comes from known BC node */

						 printf("\nWrong SYNC,PKT drop\r\n");

					 } else {

						 uint8_t  masks     = rx_buff[MASK_POS];
						 uint8_t  node_id   = rx_buff[NODE_ID_POS];
						 uint16_t pkt_id    = (rx_buff[PKT_ID_MSB_POS] << 8) | rx_buff[PKT_ID_LSB_POS];

						 uint8_t data[SENSOR_PLD_BYTES];
						 memcpy(data, &rx_buff[SMPL_DATA_POS], SENSOR_PLD_BYTES);

						 int16_t rssi       = (int16_t) ((rx_buff[RSSI_POS] << 8) | rx_buff[RSSI_POS + 1]);
						 uint8_t bc_id1     = rx_buff[BC_ID1_POS];
						 uint8_t last_bc_id = rx_buff[rx_buff_len-1];

						 uint8_t tx_bc_id   = 0;
						 if (rx_buff_len >= ENV_NODE_PYL_SIZE + RSSI_BYTES + 2*BC_ID_BYTES) tx_bc_id = rx_buff[rx_buff_len-2];

						 printf("\n###### RX PKT ######\r\n");
						 printf("       SYNC: 0x%04X\r\n", sync);
						 printf("        MSK: %d\r\n", masks);
						 printf("    NODE ID: %d\r\n", node_id);
						 printf("     PKT ID: %d\r\n", pkt_id);
						 printf("       RSSI: %i\r\n", rssi);
						 printf("  1st BC ID: %d\r\n", bc_id1);
						 printf("   Tx BC ID: %d\r\n", tx_bc_id);
						 printf(" last BC ID: %d\r\n", last_bc_id);

						 // plot payload:
						 printf(" PYLD: ");
						 for(uint8_t i = 0; i < SENSOR_PLD_BYTES; i++){
							 printf("0x%02X ", data[i]);
						 }
						 printf("\r\n");

					 }

				 }

				 /* set RFM95 back to continuous RX mode */
				 if(!rfm95_enter_rx_mode(&rfm95_handle)) printf("RFM err entering RX\r\n");

			 }
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RFM95_RST_GPIO_Port, RFM95_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RFM95_CS_GPIO_Port, RFM95_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Test_GPIO_Port, Test_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 PC6 PC7
                           PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 PA6
                           PA7 PA8 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB12 PB13 PB14
                           PB15 PB4 PB6 PB7
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9;
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

/* Setup the RTC wake-up counter to generate an interrupt event
 * used to schedule the next transmission within a time window
 * raging from [min, max] milliseconds.
 *  -> If the computed time is smaller than 5 ms the RTC event is skipped and
 *     the transmission EVT flag (SYS_EVT_TX_PENDING) is immediately set.
 *  -> If a WKUP timer is already running, the shortest between remaining time and
 *     the new computed time is kept
 *
 * SET MIN = MAX = 0 to skip the wait time and immediately schedule a new transmission
 * event.
 *
 * @param uint16_t  max represents maximum time (in ms) for the random wait
 * @param uint16_t  min represents minimum time (in ms) for the random wait
 *
 * @return uint16_t in range [min, max]
 */
static void schedule_tx_evt(uint16_t min, uint16_t max){
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

  // get random time between [min, max] (in milliseconds)
  uint16_t t_rand = random_wait(min, max);

  if (t_rand <= 5) {
    /*
     * if less than 5 ms (10 RTC ticks) are computed, skip the timer
     * and directly set a TX event
     */
    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

    h_sys.evt_flags |= SYS_EVT_TX_PENDING;
    return;
  }

  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);  // latch time

  // new EVT time in milliseconds
  uint32_t now_seconds = sTime.Hours * 3600 + sTime.Minutes * 60 + sTime.Seconds;
  uint16_t milliseconds = (uint16_t)(((sTime.SecondFraction - sTime.SubSeconds) * 1000U) / (sTime.SecondFraction + 1));
  uint32_t new_evt_time = now_seconds*1000 + milliseconds + t_rand;

  if (RTC->CR & RTC_CR_WUTE) { /* RTC WKUP already running */

    // If running timer is close to trigger -> wait for it
    if (tx_evt_time <= new_evt_time + 10) return;

    // otherwise block previous timer and start a new one with shorter WKUP CNT
    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

  }

  tx_evt_time = new_evt_time;

  // compute corresponding RTC corresponding number of ticks
  uint32_t wakeUpCounter = (uint32_t) ((t_rand * LSE_CLK / 16u)/(1000u));

  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, wakeUpCounter, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
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
	HAL_ResumeTick();

	// Enable LPTIM again:
	HAL_LPTIM_Counter_Start_IT(&hlptim1, 0xFFFF);
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
  h_sys.evt_flags |= SYS_EVT_TX_PENDING;
}

// GPIO external interrupts callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	// Events on RFM95 interrupt pins
  if (GPIO_Pin == RFM95_DIO0_Pin) {
    rfm95_on_interrupt(&rfm95_handle, RFM95_INTERRUPT_DIO0);

    // something received
    if(rfm95_handle.rfm_status == RXCONTIN_MODE) h_sys.evt_flags |= SYS_EVT_RX_PENDING;

  } else if (GPIO_Pin == RFM95_DIO1_Pin) {
    rfm95_on_interrupt(&rfm95_handle, RFM95_INTERRUPT_DIO1);
  } else if (GPIO_Pin == RFM95_DIO5_Pin) {
    rfm95_on_interrupt(&rfm95_handle, RFM95_INTERRUPT_DIO5);
  } else if (GPIO_Pin == B1_Pin) {
  	tx_evt = true;
  }
}

// ----------------------------------------------------------------------------------------



// --------------------------------- DEBUG FUNCTIONS --------------------------------------

void build_pkt(uint16_t sync, uint8_t mask, uint8_t nodeID, uint16_t pktID, uint8_t* sensors, uint8_t sens_len,
		           int16_t rssi, uint8_t* bc_seq, uint8_t bc_seq_len, uint8_t* t_buff, uint8_t* t_buff_len){

	t_buff[SYNC_WORD_POS]   = (uint8_t)((sync >> 8) & 0xFF);
	t_buff[SYNC_WORD_POS+1] = (uint8_t)( sync & 0xFF);
	t_buff[MASK_POS]        = mask;
	t_buff[NODE_ID_POS]     = nodeID;
	t_buff[PKT_ID_MSB_POS]  = (uint8_t)((pktID >> 8) & 0xFF);
	t_buff[PKT_ID_LSB_POS]  = (uint8_t)(pktID & 0xFF);

	memcpy(&t_buff[SMPL_DATA_POS],sensors, sens_len);

	*t_buff_len = ENV_NODE_PYL_SIZE;

	if(bc_seq_len != 0){

		t_buff[RSSI_POS]        = (uint8_t)((rssi >> 8) & 0xFF);
		t_buff[RSSI_POS+1]      = (uint8_t)(rssi & 0xFF);

		memcpy(&t_buff[BC_ID1_POS],bc_seq,bc_seq_len);

		*t_buff_len += (RSSI_BYTES + bc_seq_len);

	}

}

void print_LL(){

  if (h_tx.ll_status == LL_EMPTY){

    printf("\n\nIns Idx %i\r\n\n", h_tx.ins_idx);

    for(int i = 0; i < BUFF_FIFO_SIZE; i++){
      printf("[");
      printf("%i->",i);
      printf("%i]  ",h_tx.ll_buff[i].next_free_idx);
    }

    printf("\r\n");
    return;
  }

  uint16_t i = h_tx.head;
  while(i != LL_IDX_IS_TAIL){
    printf("[");
    printf("%i<-",h_tx.ll_buff[i].prev_element_idx);
    printf("%i->",h_tx.ll_buff[i].idx_rx_buff);
    printf("%i]  ",h_tx.ll_buff[i].next_element_idx);

    i = h_tx.ll_buff[i].next_element_idx;
  }

  printf("\r\n");
}

/*
 * Function used to print on UART serial (DEBUG)
 */
int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
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
