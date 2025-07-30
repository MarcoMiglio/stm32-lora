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
#include "sys_settings.h"
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
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_SPI3_Init();
  MX_LPTIM1_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */

  // needed after programming -> avoid conflicts with sleep mode
  HAL_Delay(2000);

  // init RX and TX handler
  init_buffers(&h_buffs);

  // init RF in RX mode
  if (!init_rfm()) printf("Error during RFM initialization\r\n");

  h_sys.evt_flags |= SYS_EVT_RX_PENDING;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* System state machine */
    switch (h_sys.state){

      case SYS_RDY:         /* Sys rdy -> submit new task or sleep */

        /* atomicity ensured during task registration */
        HAL_SuspendTick();
        __disable_irq();

        /* check pending flags */
        if (h_sys.evt_flags & SYS_EVT_RX_PENDING){         // RX event is pending

          h_sys.state = SYS_HANDLE_RX;
          h_sys.evt_flags &= ~SYS_EVT_RX_PENDING;

        } else if (h_sys.evt_flags & SYS_EVT_TX_PENDING){  // TX event is pending

          h_sys.state = SYS_HANLDE_TX;
          h_sys.evt_flags &= ~SYS_EVT_TX_PENDING;

        } else if (h_sys.evt_flags & SYS_EVT_ERROR){       // ERR event is pending

          // RFU...
          h_sys.state = SYS_HANDLE_ERR;
          h_sys.evt_flags &= ~SYS_EVT_ERROR;

        } else if (h_sys.evt_flags & SYS_EVT_TIMEOUT){     // TIMEOUT event is pending

          // RFU...
          h_sys.state = SYS_HANDLE_TIMEOUT;
          h_sys.evt_flags &= ~SYS_EVT_TIMEOUT;

        } else {
          // No events pending -> back to sleep
          enterStopMode();
        }

        /* end of atomic block */
        HAL_ResumeTick();
        __enable_irq();

        break;

      case SYS_HANDLE_RX:        /* Sys handle RX event */

        printf("handle RX\r\n");

        app_flags = on_rx_event(&rfm95_handle, &h_buffs);

        if (app_flags.err_flags) {           /* If any error occurred */

          if (app_flags.err_flags & EVT_RFM_SPI_ERR) {
            printf("SPI ERROR\r\n");

            // reset RFM and restart in RX mode
            reset_rfm(&rfm95_handle);
            init_rfm();
          }
          if (app_flags.err_flags & EVT_RFM_RX_ERR) {

            /*
             * RX error can happen if:
             * - CRC error is detected -> error on modulation
             * - No bytes received due to interferance
             */

            // PKT dropped, do nothing...

          }
          if (app_flags.err_flags & EVT_RX_FIFO_FULL) {
            // TODO
            printf("RX FIFO Full\r\n");
          }
          if (app_flags.err_flags & EVT_BAD_PKT_FORMAT) {
            // TODO
            printf("BAD PKT Full\r\n");
          }

        } else if (app_flags.status_flags) {  /* If status flags are present */

          // Process event flags here:
          if (app_flags.status_flags & EVT_SCHEDULE_TX) {

            printf("NEW PKT\r\n");
            /*
             *  New PKT pushed in the RX FIFO:
             *  Schedule TX event, and TX
             */
            schedule_tx_evt(MIN_WAIT_TIME_1, MAX_WAIT_TIME_1);

          }

        } else {                              /* No events pending */
          // no events pending -> do nothing
        }

        // make system ready again
        h_sys.state = SYS_RDY;
        break;

      case SYS_HANLDE_TX:        /* Sys handle TX event */

        printf("handle TX\r\n");

        HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

        app_flags = on_tx_event(&rfm95_handle, &h_buffs);

        if (app_flags.err_flags) {           /* If any error occurred */

          if (app_flags.err_flags & EVT_RFM_SPI_ERR) {
            printf("SPI ERROR\r\n");

            // reset RFM and restart in RX mode
            reset_rfm(&rfm95_handle);
            init_rfm();
          }
          // RFU... add other flags

        } else if (app_flags.status_flags) {  /* If status flags are present */

          // Process event flags here:
          if(app_flags.status_flags & EVT_RFM_MODEM_RX) {

            printf("Modem RX -> reschedule \r\n");
            //  RFM is receiving something wait for the end of RX event
            schedule_tx_evt(MIN_WAIT_TIME_1, MAX_WAIT_TIME_1);

          } else if (app_flags.status_flags & EVT_TX_FIFO_EMPTY){

            printf("NO TX Events\r\n");
            // Do nothing...

          } else if (app_flags.status_flags & EVT_SCHEDULE_PRI_TX){

            printf("PRI TX PKTs\r\n");
            // Other new PKTs are waiting for 1st TX
            schedule_tx_evt(MIN_WAIT_TIME_1, MAX_WAIT_TIME_1);

          } else if (app_flags.status_flags & EVT_SCHEDULE_TX){

            printf("Normal TX PKTs\r\n");
            // Schedule event for PKT retransmission mechanism
            schedule_tx_evt(MIN_WAIT_TIME, MAX_WAIT_TIME);

          } else {
            // RFU...
          }

        } else {                              /* No events pending */
          // no events pending -> do nothing
        }

        // make system ready again
        h_sys.state = SYS_RDY;
        break;

      case SYS_HANDLE_ERR:       /* Sys handle ERR event */

        // RFU...

        // make system ready again
        h_sys.state = SYS_RDY;
        break;

      case SYS_HANDLE_TIMEOUT:   /* Sys handle TIMEOUT event */

        //RFU...

        // make system ready again
        h_sys.state = SYS_RDY;
        break;

      default:
        h_sys.state = SYS_RDY;
        break;

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
  }
}

// ----------------------------------------------------------------------------------------



// --------------------------------- DEBUG FUNCTIONS --------------------------------------

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
