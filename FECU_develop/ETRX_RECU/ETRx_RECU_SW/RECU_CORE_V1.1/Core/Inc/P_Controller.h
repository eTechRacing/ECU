/*
 * P_Controller.h
 *
 *  Created on: Jun 1, 2024
 *      Author: Andreu, Bernat Co
 */

#ifndef INC_P_CONTROLLER_H_
#define INC_P_CONTROLLER_H_

/* Controller parameters */
//ACCU---------------------------------------------------------------------------------------------------
#define P_SETPOINT_ACCU 30
#define P_KP_ACCU 3

//MOTOR--------------------------------------------------------------------------------------------------
#define P_SETPOINT_MOTOR 60
#define P_KP_MOTOR 4

//INVERTER-----------------------------------------------------------------------------------------------
#define P_SETPOINT_INVERTER 30
#define P_KP_INVERTER 4

typedef struct {
		/* Setpoint */
		int setpoint;

		/* Controller gains */
		int Kp;

		/* Controller output */
		int out;

	} P_Controller;

int P_Controller_Init(P_Controller *pc);
int P_Controller_Update(P_Controller *pc, float measurement, int mode, int refri_select);

#endif /* INC_P_CONTROLLER_H_ */
