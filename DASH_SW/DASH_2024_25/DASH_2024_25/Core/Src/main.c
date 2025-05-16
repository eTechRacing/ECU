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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stm32l4xx_hal_gpio.h>
#include "FONTS/fonts.h"
#include "LCD/bmp.h"
#include "LCD/ili9488.h"
#include "LCD/lcd_io_spi.h"
#include "LCD/lcd.h"
#include "LCD/stm32_adafruit_lcd.h"
#include "DASH/etr_screens.h"
#include "DASH/etr_carstate.h"
#include "DASH/buttons.h"
#include "CAN/CAN_X_2025.h"
#include "CAN/CAN.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

osThreadId CANHandle;
osThreadId ButtonsHandle;
osThreadId DisplayHandle;
osSemaphoreId BinSemControlHandle;
/* USER CODE BEGIN PV */

		/* CAN */
uint8_t RxData[8];
CAN_RxHeaderTypeDef RxHeader;

		/* FLAGS*/
uint8_t flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
void StartCAN(void const * argument);
void StartButtons(void const * argument);
void StartDisplay(void const * argument);

/* USER CODE BEGIN PFP */

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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  		/*START CAN*/
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  Init_CAN_Filter(&hcan1);

  	  /*RULES INIT*/
  HAL_Delay(100);
  resetAllSignals();
  init_rules();


  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of BinSemControl */
  osSemaphoreDef(BinSemControl);
  BinSemControlHandle = osSemaphoreCreate(osSemaphore(BinSemControl), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of CAN */
  osThreadDef(CAN, StartCAN, osPriorityHigh, 0, 128);
  CANHandle = osThreadCreate(osThread(CAN), NULL);

  /* definition and creation of Buttons */
  osThreadDef(Buttons, StartButtons, osPriorityNormal, 0, 128);
  ButtonsHandle = osThreadCreate(osThread(Buttons), NULL);

  /* definition and creation of Display */
  osThreadDef(Display, StartDisplay, osPriorityBelowNormal, 0, 128);
  DisplayHandle = osThreadCreate(osThread(Display), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMD_LED_GPIO_Port, IMD_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, BMS_LED_Pin|BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_UP_Pin */
  GPIO_InitStruct.Pin = BUTTON_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_UP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_DOWN_Pin BUTTON_RIGHT_Pin BUTTON_LEFT_Pin BUTTON_OK_Pin */
  GPIO_InitStruct.Pin = BUTTON_DOWN_Pin|BUTTON_RIGHT_Pin|BUTTON_LEFT_Pin|BUTTON_OK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_SCK_Pin SPI_MISO_Pin SPI_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI_SCK_Pin|SPI_MISO_Pin|SPI_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_DC_Pin SPI_RST_Pin */
  GPIO_InitStruct.Pin = SPI_DC_Pin|SPI_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IMD_LED_Pin */
  GPIO_InitStruct.Pin = IMD_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMD_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BMS_LED_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = BMS_LED_Pin|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
  {
	HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &RxHeader, RxData);

			if(RxHeader.StdId == CTRL_BMS_Cell_Extremes_id){
				message_canrx_CTRL_BMS_Cell_Extremes(RxData);
			}
			else if (RxHeader.StdId == STAT_ETAS_Diagnostics_id){
				message_canrx_STAT_ETAS_Diagnostics(RxData);
			}
			else if (RxHeader.StdId == STAT_ETAS_Sync_id){
				message_canrx_STAT_ETAS_Sync(RxData);
			}
			else if (RxHeader.StdId == CTRL_ETAS_System_id){
				message_canrx_CTRL_ETAS_System(RxData);
				Screen.ActualState = Car_State;
			}
			else if (RxHeader.StdId == PROC_ETAS_TS_Data_id){
				message_canrx_PROC_ETAS_TS_Data(RxData);
			}
			else if (RxHeader.StdId == PROC_ETAS_VDC_LapTiming_id){
				message_canrx_PROC_ETAS_VDC_LapTiming(RxData);
			}
			else if (RxHeader.StdId == PROC_ETAS_VDC_Tq_id){
				message_canrx_PROC_ETAS_VDC_Tq(RxData);
			}
			else if (RxHeader.StdId == PROC_ETAS_VDC_Values_id){
				message_canrx_PROC_ETAS_VDC_Values(RxData);
			}
			else if (RxHeader.StdId == PROC_ETAS_VDC_Params_id){
				message_canrx_PROC_ETAS_VDC_Params(RxData);
			}
			else if (RxHeader.StdId == STAT_BMS_Keep_Alive_id) {
				message_canrx_STAT_BMS_Keep_Alive(RxData);
			}
			else if (RxHeader.StdId == CTRL_BMS_Accu_Data_id){
				message_canrx_CTRL_BMS_Accu_Data(RxData);

			}
			refreshGPIOs();
  }
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartCAN */
/**
  * @brief  Function implementing the CAN thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartCAN */
void StartCAN(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  message_cantx_STAT_DASH_Keep_Alive(hcan1);
	  Dash_Alive++;

	  if (PrechargeRequest == 1 && Screen.ActualState  == DASH_1_PRECHARGE) {
		  message_cantx_CTRL_DASH_Driver_Inputs(hcan1);
		  PrechargeRequest = 0;
		  HAL_Delay(100);
		  message_cantx_CTRL_DASH_Driver_Inputs(hcan1);
	  }
	  else if (RacingMode_Send == 1 && Screen.ActualState  == DASH_4_RACING_MENU) {
		  message_cantx_CTRL_DASH_Driver_Inputs(hcan1);
		  RacingMode_Send = 0;
	  }
	  else if (EnableDrive_Order == 1 && Screen.ActualState  == DASH_4_RACING_MENU) {
		  message_cantx_CTRL_DASH_Driver_Inputs(hcan1);
		  EnableDrive_Order = 0;
		  HAL_Delay(100);
		  message_cantx_CTRL_DASH_Driver_Inputs(hcan1);
	  }
	  else if (pendingButtonEvent == EVENT_ROTARY_LEFT || pendingButtonEvent == EVENT_ROTARY_RIGHT){
		  message_cantx_CTRL_DASH_Driver_Inputs(hcan1);
	  }

	  vTaskDelay(50);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartButtons */
/**
* @brief Function implementing the Buttons thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartButtons */
void StartButtons(void const * argument)
{
  /* USER CODE BEGIN StartButtons */
  /* Infinite loop */
  for(;;)
  {

	  currentButtonState_Up = HAL_GPIO_ReadPin(BUTTON_UP_GPIO_Port, BUTTON_UP_Pin);
	  currentButtonState_Down = HAL_GPIO_ReadPin(BUTTON_DOWN_GPIO_Port, BUTTON_DOWN_Pin);
	  currentButtonState_Right = HAL_GPIO_ReadPin(BUTTON_RIGHT_GPIO_Port, BUTTON_RIGHT_Pin);
	  currentButtonState_Left = HAL_GPIO_ReadPin(BUTTON_LEFT_GPIO_Port, BUTTON_LEFT_Pin);
	  currentButtonState_OK = HAL_GPIO_ReadPin(BUTTON_OK_GPIO_Port, BUTTON_OK_Pin);
	  currentRotary_Left = 0;	// Poner la funcion de HAL
	  currentRotary_Right = 0; // Poner la funcion de HAL

	  refreshButton();
	  refreshScreen();

    osDelay(10);
  }
  /* USER CODE END StartButtons */
}

/* USER CODE BEGIN Header_StartDisplay */
/**
* @brief Function implementing the Display thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplay */
void StartDisplay(void const * argument)
{
  /* USER CODE BEGIN StartDisplay */
  /* Infinite loop */
  for(;;)
  {
//	  drawScreen();
    osDelay(5);
  }
  /* USER CODE END StartDisplay */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
