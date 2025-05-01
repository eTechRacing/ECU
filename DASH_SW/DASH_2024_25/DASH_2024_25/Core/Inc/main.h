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
#define BUTTON_UP_Pin GPIO_PIN_2
#define BUTTON_UP_GPIO_Port GPIOB
#define BUTTON_DOWN_Pin GPIO_PIN_7
#define BUTTON_DOWN_GPIO_Port GPIOE
#define BUTTON_LEFT_Pin GPIO_PIN_8
#define BUTTON_LEFT_GPIO_Port GPIOE
#define BUTTON_RIGHT_Pin GPIO_PIN_10
#define BUTTON_RIGHT_GPIO_Port GPIOE
#define BUTTON_OK_Pin GPIO_PIN_12
#define BUTTON_OK_GPIO_Port GPIOE
#define SPI_SCK_Pin GPIO_PIN_10
#define SPI_SCK_GPIO_Port GPIOC
#define SPI_MISO_Pin GPIO_PIN_11
#define SPI_MISO_GPIO_Port GPIOC
#define SPI_MOSI_Pin GPIO_PIN_12
#define SPI_MOSI_GPIO_Port GPIOC
#define SPI_CS_Pin GPIO_PIN_0
#define SPI_CS_GPIO_Port GPIOD
#define SPI_DC_Pin GPIO_PIN_6
#define SPI_DC_GPIO_Port GPIOB
#define SPI_RST_Pin GPIO_PIN_7
#define SPI_RST_GPIO_Port GPIOB
#define IMD_LED_Pin GPIO_PIN_9
#define IMD_LED_GPIO_Port GPIOB
#define TS_LED_Pin GPIO_PIN_0
#define TS_LED_GPIO_Port GPIOE
#define BUZZER_Pin GPIO_PIN_1
#define BUZZER_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
