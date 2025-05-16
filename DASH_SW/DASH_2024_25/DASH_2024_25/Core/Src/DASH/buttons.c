/*
 * buttons.c
 *
 *  Created on: Mar 6, 2025
 *      Author: cunio
 */

#include <stdint.h>

#include "DASH/etr_screens.h"
#include "DASH/buttons.h"
#include "CAN/CAN_X_2025.h"
#include "DASH/etr_carstate.h"

// variables globales -----------------------------------------------------------------------------------------

uint8_t currentButtonState_Up;
uint8_t currentButtonState_Down;
uint8_t currentButtonState_Left;
uint8_t currentButtonState_Right;
uint8_t currentButtonState_OK;
uint8_t currentRotaryState_1;
uint8_t currentRotaryState_2;


uint8_t lastButtonState_Up;
uint8_t lastButtonState_Down;
uint8_t lastButtonState_Left;
uint8_t lastButtonState_Right;
uint8_t lastButtonState_OK;
uint8_t lastRotaryState_1;
uint8_t lastRotaryState_2;

uint8_t pendingButtonEvent;

// funciones de buttons ----------------------------------------------------------------------------------------


void refreshButton(void){

	if ((currentButtonState_Up != lastButtonState_Up) && (lastButtonState_Up != 1)) {
		 pendingButtonEvent = EVENT_BUTTON_UP;
	}

	if ((currentButtonState_Down != lastButtonState_Down) && (lastButtonState_Down != 1)) {
		pendingButtonEvent = EVENT_BUTTON_DOWN;
	}

	if ((currentButtonState_Left != lastButtonState_Left) && (lastButtonState_Left != 1)) {
		pendingButtonEvent = EVENT_BUTTON_LEFT;
	}

	if ((currentButtonState_Right != lastButtonState_Right) && (lastButtonState_Right != 1)) {
		pendingButtonEvent = EVENT_BUTTON_RIGHT;
	}

	if ((currentButtonState_OK != lastButtonState_OK) && (lastButtonState_OK != 1)) {
		pendingButtonEvent = EVENT_BUTTON_OK;
	}

	if (currentRotaryState_1 != lastRotaryState_1) {
		pendingButtonEvent = EVENT_ROTARY_1;
	}

	if (currentRotaryState_2 != lastRotaryState_2) {
		pendingButtonEvent = EVENT_ROTARY_2;
	}
	lastButtonState_Up = currentButtonState_Up;
	lastButtonState_Down = currentButtonState_Down;
	lastButtonState_Left = currentButtonState_Left;
	lastButtonState_Right = currentButtonState_Right;
	lastButtonState_OK = currentButtonState_OK;
	lastRotaryState_1 = currentRotaryState_1;
	lastRotaryState_2 = currentRotaryState_2;
}

uint8_t readRotarySwitch1(void) {
    uint8_t bit3 = HAL_GPIO_ReadPin(SW1_1_GPIO_Port, SW1_1_Pin); // MSB
    uint8_t bit2 = HAL_GPIO_ReadPin(SW1_2_GPIO_Port, SW1_2_Pin);
    uint8_t bit1 = HAL_GPIO_ReadPin(SW1_3_GPIO_Port, SW1_3_Pin);
    uint8_t bit0 = HAL_GPIO_ReadPin(SW1_4_GPIO_Port, SW1_4_Pin); // LSB

    uint8_t value = (bit3 << 3) | (bit2 << 2) | (bit1 << 1) | bit0;

    return value;
}

uint8_t readRotarySwitch2(void) {
    uint8_t bit3 = HAL_GPIO_ReadPin(SW2_1_GPIO_Port, SW2_1_Pin); // MSB
    uint8_t bit2 = HAL_GPIO_ReadPin(SW2_2_GPIO_Port, SW2_2_Pin);
    uint8_t bit1 = HAL_GPIO_ReadPin(SW2_3_GPIO_Port, SW2_3_Pin);
    uint8_t bit0 = HAL_GPIO_ReadPin(SW2_4_GPIO_Port, SW2_4_Pin); // LSB

    uint8_t value = (bit3 << 3) | (bit2 << 2) | (bit1 << 1) | bit0;

    return value;
}
