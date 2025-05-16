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
extern uint8_t currentRotaryState_1;
extern uint8_t currentRotaryState_2;

extern uint8_t lastButtonState_Up;
extern uint8_t lastButtonState_Down;
extern uint8_t lastButtonState_Left;
extern uint8_t lastButtonState_Right;
extern uint8_t lastButtonState_OK;
extern uint8_t lastRotaryState_1;
extern uint8_t lastRotaryState_2;

extern uint8_t pendingButtonEvent;

// definiciones -----------------------------------------------------------------------------------------------


#define EVENT_NONE       	0
#define EVENT_BUTTON_UP  	1
#define EVENT_BUTTON_DOWN 	2
#define EVENT_BUTTON_LEFT 	3
#define EVENT_BUTTON_RIGHT 	4
#define EVENT_BUTTON_OK   	5
#define EVENT_ROTARY_1		6
#define EVENT_ROTARY_2		7

// definicion de funciones --------------------------------------------------------------------------------------

void refreshButton(void);
uint8_t readRotarySwitch1(void);
uint8_t readRotarySwitch2(void);




#endif /* INC_DASH_BUTTONS_H_ */
