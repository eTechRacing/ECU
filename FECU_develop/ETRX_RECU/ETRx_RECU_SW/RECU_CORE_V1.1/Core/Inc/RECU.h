/*
 * RECU.h
 *
 *  Created on: Jun 1, 2024
 *      Author: Andreu
 */

#ifndef INC_RECU_H_
#define INC_RECU_H_


#include "main.h"
#include "stdlib.h"

#define TSMS_TSMP_PORT GPIOB
#define TSMS_TSMP_PIN GPIO_PIN_2

#define RIGHT_TS_PORT GPIOE
#define RIGHt_TS_PIN GPIO_PIN_7

#define LEFT_TS_PORT GPIOE
#define LEFT_TS_PIN GPIO_PIN_8

#define HVBOX_PORT GPIOE
#define HVBOX_PIN GPIO_PIN_9

#define HVD_PORT GPIOE
#define HVD_PIN GPIO_PIN_10

uint16_t highest_temp_calc(uint16_t Temp_R,uint16_t Temp_L);

#endif /* INC_RECU_H_ */
