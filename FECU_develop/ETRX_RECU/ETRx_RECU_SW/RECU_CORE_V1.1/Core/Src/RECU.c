/*
 * RECU.c
 *
 *  Created on: Jun 1, 2024
 *      Author: Andreu
 */

#include "RECU.h"

uint16_t highest_temp_calc(uint16_t Temp_R,uint16_t Temp_L){
	if(Temp_R>Temp_L){
		return Temp_R;
	}
	else{
		return Temp_L;
	}
}


