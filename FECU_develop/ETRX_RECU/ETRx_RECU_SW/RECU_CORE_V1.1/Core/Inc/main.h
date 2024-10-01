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
#define Susp_R_L_Pin GPIO_PIN_2
#define Susp_R_L_GPIO_Port GPIOC
#define Susp_R_R_Pin GPIO_PIN_3
#define Susp_R_R_GPIO_Port GPIOC
#define TSMS_TSMP_Pin GPIO_PIN_2
#define TSMS_TSMP_GPIO_Port GPIOB
#define Right_TS_Pin GPIO_PIN_7
#define Right_TS_GPIO_Port GPIOE
#define Left_TS_Pin GPIO_PIN_8
#define Left_TS_GPIO_Port GPIOE
#define HVBox_Pin GPIO_PIN_9
#define HVBox_GPIO_Port GPIOE
#define HVD_Pin GPIO_PIN_10
#define HVD_GPIO_Port GPIOE
#define MOTOR_PWM_Pin GPIO_PIN_7
#define MOTOR_PWM_GPIO_Port GPIOC
#define ACCU_PWM_Pin GPIO_PIN_9
#define ACCU_PWM_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
