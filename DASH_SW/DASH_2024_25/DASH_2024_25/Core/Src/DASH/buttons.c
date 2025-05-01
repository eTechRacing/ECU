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

uint8_t lastButtonState_Up;
uint8_t lastButtonState_Down;
uint8_t lastButtonState_Left;
uint8_t lastButtonState_Right;
uint8_t lastButtonState_OK;

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

	if ((currentButtonState_OK != lastButtonState_OK) && (lastButtonState_Right != 1)) {
		pendingButtonEvent = EVENT_BUTTON_OK;
	}

	lastButtonState_Up = currentButtonState_Up;
	lastButtonState_Down = currentButtonState_Down;
	lastButtonState_Left = currentButtonState_Left;
	lastButtonState_Right = currentButtonState_Right;
	lastButtonState_OK = currentButtonState_OK;
}


