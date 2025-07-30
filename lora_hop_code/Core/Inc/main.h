/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/*
 * Possible system states
 */
typedef enum{
  SYS_HANDLE_RX,
  SYS_HANLDE_TX,
  SYS_HANDLE_ERR,
  SYS_HANDLE_TIMEOUT,
  SYS_RDY
} Sys_state;

/*
 * Possible pending events (set on interrupt events)
 */
typedef enum {
  SYS_EVT_NO_PENDING    = (1 << 0),
  SYS_EVT_RX_PENDING    = (1 << 1),
  SYS_EVT_TX_PENDING    = (1 << 2),
  SYS_EVT_ERROR         = (1 << 3),
  SYS_EVT_TIMEOUT       = (1 << 4)
} Sys_event_flags;

/*
 * System handler -> manage status updates and possible events
 */
typedef struct{
  volatile Sys_event_flags evt_flags;
  Sys_state state;
} Sys_handler;


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define RFM95_DIO0_Pin GPIO_PIN_9
#define RFM95_DIO0_GPIO_Port GPIOA
#define RFM95_DIO0_EXTI_IRQn EXTI9_5_IRQn
#define RFM95_DIO1_Pin GPIO_PIN_10
#define RFM95_DIO1_GPIO_Port GPIOA
#define RFM95_DIO1_EXTI_IRQn EXTI15_10_IRQn
#define RFM95_DIO5_Pin GPIO_PIN_11
#define RFM95_DIO5_GPIO_Port GPIOA
#define RFM95_DIO5_EXTI_IRQn EXTI15_10_IRQn
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define RFM95_RST_Pin GPIO_PIN_15
#define RFM95_RST_GPIO_Port GPIOA
#define RFM95_CS_Pin GPIO_PIN_2
#define RFM95_CS_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Test_Pin GPIO_PIN_5
#define Test_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define LSE_CLK (1<<15)
#define MILLISECONDS_IN_S 1000u
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
