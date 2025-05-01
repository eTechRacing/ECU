/*
 * 	etr_carstate.c
 *
 *  Created on: Feb 25, 2025
 *      Author: cunio
 */

#include "DASH/etr_carstate.h"
#include "DASH/buttons.h"
#include "stdint.h"
#include "LCD/stm32_adafruit_lcd.h"
#include "DASH/etr_carstate.h"
#include "CAN/CAN_X_2025.h"
#include <stdio.h>


uint8_t RacingMode_Send;


const int NUM_SCREENS_PER_STATE[] = {
    3,  // DASH_0_ETR
    3,  // DASH_3_PRECHARGE
    1,  // DASH_6_PRECHARGE_STATUS
    1,  // DASH_9_PRECHARGE_FINISHED
    11, // DASH_12_RACING_MENU
    1,  // DASH_14_INVERTERS
    5,  // DASH_15_RACING_MODE
    1   // DASH_21_ERROR
};

DASH_State Screen = {DASH_0_ETR, SCREEN_1, SCREEN_1};

void GlobalVariableReset(void){
	Driver = 0;
	RacingMode = 0;
	Dash_Alive = 0;
	EnableDrive_Order = 0;
	RacingMode_Send = 0;
	PrechargeRequest = 0;
}

void init_rules(void){
	HAL_GPIO_WritePin(IMD_LED_GPIO_Port, IMD_LED_Pin, 1);
	HAL_GPIO_WritePin(TS_LED_GPIO_Port, TS_LED_Pin, 1);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);

	HAL_Delay(2500);

	HAL_GPIO_WritePin(IMD_LED_GPIO_Port, IMD_LED_Pin, 0);
	HAL_GPIO_WritePin(TS_LED_GPIO_Port, TS_LED_Pin, 0);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
}

	// haces la pantalla estatica, y los datos se ponen en la otra funcion
void refreshScreen(void) {

    		// RESETEA LA PANTALLA SI SE CAMBIA EL ESTADO
    if (Screen.PreviousState != Screen.ActualState){

    	Screen.ActualScreen = SCREEN_1;
    	if (Screen.ActualState == DASH_12_RACING_MENU){
    		Driver = 1;
    		RacingMode = 1;
    	}
    	Screen.PreviousState = Screen.ActualState;
    }

    switch(Screen.ActualState) {

        case DASH_0_ETR:

            if (pendingButtonEvent == EVENT_BUTTON_DOWN) {
            	if(Screen.ActualScreen == SCREEN_3){
            		Screen.ActualScreen = SCREEN_1;
            	} else {
            		Screen.ActualScreen ++;
            	}
            }
            if (pendingButtonEvent == EVENT_BUTTON_UP) {
            	if(Screen.ActualScreen == SCREEN_1){
            		Screen.ActualScreen = SCREEN_3;
            	} else {
            		Screen.ActualScreen --;
            	}
            }

            if (pendingButtonEvent == EVENT_BUTTON_RIGHT) {

            }
            if (pendingButtonEvent == EVENT_BUTTON_LEFT) {

            }
            if (pendingButtonEvent == EVENT_BUTTON_OK) {

            }

            break;

        case DASH_3_PRECHARGE:

            if (pendingButtonEvent == EVENT_BUTTON_DOWN) {

            }
            if (pendingButtonEvent == EVENT_BUTTON_UP) {

            }
            if (pendingButtonEvent == EVENT_BUTTON_RIGHT) {

            }
            if (pendingButtonEvent == EVENT_BUTTON_LEFT) {

            }
            if (pendingButtonEvent == EVENT_BUTTON_OK) {
            	PrechargeRequest = 1;
            }
            break;

        case DASH_6_PRECHARGE_STATUS:

            if (pendingButtonEvent == EVENT_BUTTON_DOWN) {

            }
            if (pendingButtonEvent == EVENT_BUTTON_UP) {

            }
            if (pendingButtonEvent == EVENT_BUTTON_RIGHT) {

            }
            if (pendingButtonEvent == EVENT_BUTTON_LEFT) {

            }
            if (pendingButtonEvent == EVENT_BUTTON_OK) {

            }
            break;

        case DASH_9_PRECHARGE_FINISHED:

            if (pendingButtonEvent == EVENT_BUTTON_DOWN) {

            }
            if (pendingButtonEvent == EVENT_BUTTON_UP) {

            }
            if (pendingButtonEvent == EVENT_BUTTON_RIGHT) {

            }
            if (pendingButtonEvent == EVENT_BUTTON_LEFT) {

            }
            if (pendingButtonEvent == EVENT_BUTTON_OK) {

            }
            break;

        case DASH_12_RACING_MENU:

            if (pendingButtonEvent == EVENT_BUTTON_DOWN) {
            	if (Screen.ActualScreen >= SCREEN_5){
            		Screen.ActualScreen = SCREEN_1;
            		RacingMode = 1;
            	} else {
            		Screen.ActualScreen ++;
            		RacingMode ++;
            	}
            }

            if (pendingButtonEvent == EVENT_BUTTON_UP) {
            	if (Screen.ActualScreen == SCREEN_1){
            		Screen.ActualScreen = SCREEN_5;
            		RacingMode = 5;
            	} else {
            		Screen.ActualScreen --;
            		RacingMode --;
            	}
            }

            if (pendingButtonEvent == EVENT_BUTTON_RIGHT) {
            	if (Screen.ActualScreen >= SCREEN_1 && Screen.ActualScreen <= SCREEN_5){
            		Screen.ActualScreen = SCREEN_6;
            		Driver = 1;
            	}

            	if (Screen.ActualScreen == SCREEN_10){
            		Screen.ActualScreen = SCREEN_6;
            		Driver = 1;
            	} else {
            		Screen.ActualScreen ++;
            	}
            }

            if (pendingButtonEvent == EVENT_BUTTON_LEFT) {
            	if (Screen.ActualScreen >= SCREEN_1 && Screen.ActualScreen <= SCREEN_5){
            		Screen.ActualScreen = SCREEN_10;
            		Driver = 5;
            	}

            	if (Screen.ActualScreen == SCREEN_6){
            		Screen.ActualScreen = SCREEN_10;
            		Driver = 5;
            	} else {
            		Screen.ActualScreen --;
            	}
            }

            if (pendingButtonEvent == EVENT_BUTTON_OK) {

            	if (Screen.ActualScreen == SCREEN_11){
            		EnableDrive_Order = 1;
            	}
            	if (Screen.ActualScreen >= SCREEN_6 && Screen.ActualScreen <= SCREEN_10){
            		Screen.ActualScreen = SCREEN_11;
            		RacingMode_Send = 1;
            	}


            }

            break;

        case DASH_14_INVERTERS:

            if (pendingButtonEvent == EVENT_BUTTON_RIGHT) {

            }
            break;

        case DASH_15_RACING_MODE:

            if (pendingButtonEvent == EVENT_BUTTON_RIGHT) {

            }
            break;

        case DASH_21_ERROR:

            if (pendingButtonEvent == EVENT_BUTTON_RIGHT) {
            }
            break;

    }


}


// Declaramos la función que dibuja la pantalla
void drawScreen(void) {
    switch (Screen.ActualState) {
        case DASH_0_ETR:
            switch (Screen.ActualScreen) {
                case SCREEN_1:
                    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
                    BSP_LCD_SetFont(&Font24);
                    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
                    BSP_LCD_Clear(LCD_COLOR_WHITE);

                    BSP_LCD_DisplayStringAt(0, 100, "CAR STATE 0", CENTER_MODE);
                	break;
                default:
                	break;
            }
            break;

        case DASH_3_PRECHARGE:
            switch (Screen.ActualScreen) {
                case SCREEN_1:
                    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
                    BSP_LCD_SetFont(&Font24);
                    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
                    BSP_LCD_Clear(LCD_COLOR_WHITE);

                    BSP_LCD_DisplayStringAt(0, 100, "PRESS OK BUTTON TO PRECHARGE", CENTER_MODE);
                    break;
                default:
                	break;
            }
            break;

        case DASH_6_PRECHARGE_STATUS:
            		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
            		BSP_LCD_SetFont(&Font24);
            		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
            		BSP_LCD_Clear(LCD_COLOR_WHITE);

            		BSP_LCD_DisplayStringAt(0, 100, "PRECHARGING...", CENTER_MODE);
            break;

        case DASH_9_PRECHARGE_FINISHED:
            		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
            		BSP_LCD_SetFont(&Font24);
            		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
            		BSP_LCD_Clear(LCD_COLOR_WHITE);

            		BSP_LCD_DisplayStringAt(0, 100, "PRECHARGE FINISHED", CENTER_MODE);
            break;

        case DASH_12_RACING_MENU:
            switch (Screen.ActualScreen) {
                case SCREEN_1:
                    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
                    BSP_LCD_SetFont(&Font24);
                    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
                    BSP_LCD_Clear(LCD_COLOR_WHITE);

                    BSP_LCD_DisplayStringAt(0, 100, "RACING MENU. PRESS OK", CENTER_MODE);
                    break;
                default:
                	break;
            }
            break;

        case DASH_14_INVERTERS:
            		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
            		BSP_LCD_SetFont(&Font24);
            		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
            		BSP_LCD_Clear(LCD_COLOR_WHITE);

            		BSP_LCD_DisplayStringAt(0, 100, "INVERTERS GETTING READY", CENTER_MODE);
            break;

        case DASH_15_RACING_MODE:
            switch (Screen.ActualScreen) {
                case SCREEN_1:
                    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
                    BSP_LCD_SetFont(&Font24);
                    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
                    BSP_LCD_Clear(LCD_COLOR_WHITE);

                    BSP_LCD_DisplayStringAt(0, 100, "VROOM VROOM", CENTER_MODE);
                    break;
                default:
                	break;
            }
            break;

        case DASH_21_ERROR:
            		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
            		BSP_LCD_SetFont(&Font24);
            		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
            		BSP_LCD_Clear(LCD_COLOR_WHITE);

            		BSP_LCD_DisplayStringAt(0, 100, "OOPSIE... SOMETHING WENT WRONG", CENTER_MODE);
            break;

        default:
            		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
            		BSP_LCD_SetFont(&Font24);
            		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
            		BSP_LCD_Clear(LCD_COLOR_WHITE);

            		BSP_LCD_DisplayStringAt(0, 100, "CAR STATE ???", CENTER_MODE);
            break;
    }
}
