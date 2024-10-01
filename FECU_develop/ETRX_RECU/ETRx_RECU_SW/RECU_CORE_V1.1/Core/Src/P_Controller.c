/*
 * PID.c
 *
 *  Created on: Jun 1, 2024
 *      Author: Andreu, Bernat Co
 */

#include <P_Controller.h>
#include "stdio.h"
#include "stdint.h"

int P_Controller_Init(P_Controller *pc) {
	pc->out = 65;
	return pc->out;
}

int P_Controller_Update(P_Controller *pc, float measurement, int mode, int refri_select) {

	uint8_t limit_h;

	//Refri_select
	switch(refri_select)
	{
	case 0:
		pc->setpoint=P_SETPOINT_ACCU;
		pc->Kp=P_KP_ACCU;
		limit_h=50;
		break;
	case 1:
		pc->setpoint=P_SETPOINT_MOTOR;
		pc->Kp=P_KP_MOTOR;
		limit_h=100;
		break;
	case 2:
		pc->setpoint=P_SETPOINT_INVERTER;
		pc->Kp=P_KP_INVERTER;
		limit_h=100;
		break;
	}

	/*
	* Error signal
	*/
    float error = measurement - pc->setpoint;

	/*
	* Proportional
	*/
    float proportional = pc->Kp * error;

	/*
	* Compute output and apply limits
	*/
    if(mode==1){
    	pc->out=10;
    }
    else if (mode==0){
    	pc->out = proportional;
    	if (pc->out > limit_h) {
    		pc->out = limit_h;
    	} else if (pc->out < 5) {
    	    pc->out = 5;
    	}
    }
    else if (mode==2){
    	pc->out=limit_h;
    }
	/* Return controller output */
    return pc->out;
}
