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
#include "stm32f4xx_hal.h"

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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_14
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_15
#define LED2_GPIO_Port GPIOC
#define INTAN_MISO__Pin GPIO_PIN_1
#define INTAN_MISO__GPIO_Port GPIOA
#define SD_CS_Pin GPIO_PIN_2
#define SD_CS_GPIO_Port GPIOA
#define DAC1_Pin GPIO_PIN_4
#define DAC1_GPIO_Port GPIOA
#define SD_SCK_Pin GPIO_PIN_5
#define SD_SCK_GPIO_Port GPIOA
#define SD_MISO_Pin GPIO_PIN_6
#define SD_MISO_GPIO_Port GPIOA
#define SD_MOSI_Pin GPIO_PIN_7
#define SD_MOSI_GPIO_Port GPIOA
#define INTAN_SCK____Pin GPIO_PIN_0
#define INTAN_SCK____GPIO_Port GPIOB
#define INTAN_SCK__Pin GPIO_PIN_10
#define INTAN_SCK__GPIO_Port GPIOB
#define hj_role_Pin GPIO_PIN_12
#define hj_role_GPIO_Port GPIOB
#define INTAN_SCK___Pin GPIO_PIN_13
#define INTAN_SCK___GPIO_Port GPIOB
#define hj_mode_Pin GPIO_PIN_14
#define hj_mode_GPIO_Port GPIOB
#define INTAN_MISO_B15_Pin GPIO_PIN_15
#define INTAN_MISO_B15_GPIO_Port GPIOB
#define HJ_STATE_Pin GPIO_PIN_8
#define HJ_STATE_GPIO_Port GPIOA
#define HJ_TX_Pin GPIO_PIN_9
#define HJ_TX_GPIO_Port GPIOA
#define HJ_RX_Pin GPIO_PIN_10
#define HJ_RX_GPIO_Port GPIOA
#define INTAN_SCK_Pin GPIO_PIN_3
#define INTAN_SCK_GPIO_Port GPIOB
#define INTAN_MISO_Pin GPIO_PIN_4
#define INTAN_MISO_GPIO_Port GPIOB
#define INTAN_MOSI_Pin GPIO_PIN_5
#define INTAN_MOSI_GPIO_Port GPIOB
#define INTAN_CS_Pin GPIO_PIN_6
#define INTAN_CS_GPIO_Port GPIOB
#define INTAN_MISO1__Pin GPIO_PIN_8
#define INTAN_MISO1__GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
