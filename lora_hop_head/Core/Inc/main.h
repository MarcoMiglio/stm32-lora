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
#include "stdbool.h"
#include "sys_settings.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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

typedef struct {
    uint8_t err_flags;    // each bit is an error flag
    uint8_t status_flags; // each bit is a status flag
} events_flags;

#define EVT_RFM_SPI_ERR      (1 << 0)
#define EVT_RFM_RX_ERR       (1 << 1)
#define EVT_RX_FIFO_FULL     (1 << 2)
#define EVT_BAD_PKT_FORMAT   (1 << 3)

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
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
