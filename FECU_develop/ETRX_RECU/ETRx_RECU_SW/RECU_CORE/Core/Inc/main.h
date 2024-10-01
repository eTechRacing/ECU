/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f7xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define S5_3_Pin GPIO_PIN_1
#define S5_3_GPIO_Port GPIOC
#define S5_2_Pin GPIO_PIN_2
#define S5_2_GPIO_Port GPIOC
#define S5_1_Pin GPIO_PIN_3
#define S5_1_GPIO_Port GPIOC
#define SD_6_Pin GPIO_PIN_0
#define SD_6_GPIO_Port GPIOA
#define BL_Control_Pin GPIO_PIN_1
#define BL_Control_GPIO_Port GPIOA
#define S5_4_Pin GPIO_PIN_1
#define S5_4_GPIO_Port GPIOB
#define SD_1_Pin GPIO_PIN_2
#define SD_1_GPIO_Port GPIOB
#define SD_2_Pin GPIO_PIN_7
#define SD_2_GPIO_Port GPIOE
#define SD_3_Pin GPIO_PIN_8
#define SD_3_GPIO_Port GPIOE
#define SD_4_Pin GPIO_PIN_9
#define SD_4_GPIO_Port GPIOE
#define SD_5_Pin GPIO_PIN_10
#define SD_5_GPIO_Port GPIOE
#define PWM_FANS_Pin GPIO_PIN_7
#define PWM_FANS_GPIO_Port GPIOC
#define PWM_ACCU_Pin GPIO_PIN_9
#define PWM_ACCU_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
