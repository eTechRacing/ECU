/*
 * CAN.h
 *
 *  Created on: Feb 19, 2025
 *      Author: Pol
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "main.h"

#define ETAS_MAX_TIME 500

extern uint8_t Disconnection_ETAS;
extern uint32_t ETAS_Tick;
extern uint32_t aux_ETAS;

void Init_CAN_Filter(CAN_HandleTypeDef *hcan1);
void ETAS_disconnection_funct(void);
#endif /* INC_CAN_H_ */
