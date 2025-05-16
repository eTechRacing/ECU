/*
 * buttons.h
 *
 *  Created on: Mar 14, 2025
 *      Author: cunio
 */

#ifndef INC_DASH_BUTTONS_H_
#define INC_DASH_BUTTONS_H_

// variables globales ----------------------------------------------------------------------------------------
extern uint8_t currentButtonState_Up;
extern uint8_t currentButtonState_Down;
extern uint8_t currentButtonState_Left;
extern uint8_t currentButtonState_Right;
extern uint8_t currentButtonState_OK;

extern uint8_t lastButtonState_Up;
extern uint8_t lastButtonState_Down;
extern uint8_t lastButtonState_Left;
extern uint8_t lastButtonState_Right;
extern uint8_t lastButtonState_OK;

extern uint8_t pendingButtonEvent;

// definiciones -----------------------------------------------------------------------------------------------


#define EVENT_NONE       	0
#define EVENT_BUTTON_UP  	1
#define EVENT_BUTTON_DOWN 	2
#define EVENT_BUTTON_LEFT 	3
#define EVENT_BUTTON_RIGHT 	4
#define EVENT_BUTTON_OK   	5
#define EVENT_ROTARY_LEFT	6
#define EVENT_ROTARY_RIGHT	7

// definicion de funciones --------------------------------------------------------------------------------------

void refreshButton(void);


#endif /* INC_DASH_BUTTONS_H_ */
