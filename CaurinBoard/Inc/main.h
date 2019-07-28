/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define MISO_COMM_Pin GPIO_PIN_2
#define MISO_COMM_GPIO_Port GPIOC
#define MOSI_COMM_Pin GPIO_PIN_3
#define MOSI_COMM_GPIO_Port GPIOC
#define THERM_Pin GPIO_PIN_4
#define THERM_GPIO_Port GPIOA
#define SCK_COMM_Pin GPIO_PIN_10
#define SCK_COMM_GPIO_Port GPIOB
#define RESET_COMM_RX_Pin GPIO_PIN_12
#define RESET_COMM_RX_GPIO_Port GPIOB
#define SS_COMM_RX_Pin GPIO_PIN_13
#define SS_COMM_RX_GPIO_Port GPIOB
#define DIO0_RX_Pin GPIO_PIN_14
#define DIO0_RX_GPIO_Port GPIOB
#define DIO0_RX_EXTI_IRQn EXTI15_10_IRQn
#define TEST_LED_Pin GPIO_PIN_6
#define TEST_LED_GPIO_Port GPIOC
#define RESET_COMM_TX_Pin GPIO_PIN_9
#define RESET_COMM_TX_GPIO_Port GPIOC
#define SS_COMM_TX_Pin GPIO_PIN_8
#define SS_COMM_TX_GPIO_Port GPIOA
#define DIO0_COMM_TX_Pin GPIO_PIN_9
#define DIO0_COMM_TX_GPIO_Port GPIOA
#define DIO0_COMM_TX_EXTI_IRQn EXTI9_5_IRQn
#define DEPLOY_Pin GPIO_PIN_2
#define DEPLOY_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
