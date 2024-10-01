/*
 * eTechRacing_Filters.c
 *
 *  Created on: May 24, 2024
 *      Author: Andreu
 */
// First-order filter
//=========================================================================
/**
 * @brief Initializes the first-order filter.
 *
 * @param v Pointer to the first-order filter structure.
 *
 * @note This function initializes the first-order filter.
 */

#include "eTechRacing_Filters.h"
#include "stdio.h"
#include "stdint.h"

//void filtreLP_init(filtreLP_struct *v)
//{
//    float aux;
//    aux = v->fc* v->Ts;                                  // fc*Ts (fc in kHz)
//    aux = aux * 1000.0;                                  // Convert kHz to Hz
//    aux = PI2 * aux;                                      // 2*pi*fc*Ts
//    v->alfa = aux/(aux + 1.0);                            // 2*pi*fc*Ts/(2*pi*fc*Ts + 1)
//}
//
///**
// * @brief Calculates the first-order filter.
// *
// * @param v Pointer to the first-order filter structure.
// *
// * @note This function calculates the first-order filter.
// */
//void filtreLP_calc(filtreLP_struct *v)
//{
//    if (v->enable)
//    {
//        v->out = (v->alfa * (v->in - v->out)) + v->out;    // Filter out(k) = alfa*in(k) + (1-alfa)*out(k-1)
//    }
//    else
//    {
//        v->out = v->in;        // Without filter. Out(k) = In(k)
//    }
//}

void applyLowPassFilter(uint16_t *value)
{
	float fc = 0.5f;  // Cutoff frequency in kHz
	float Ts = 0.001f;  // Sampling time in seconds
	float alpha = (fc*Ts*1000.0*PI2)/((fc*Ts*1000.0*PI2)+1.0);
	value[0] = (uint16_t)round(alpha*value[0]+(1-alpha)*value[1]);
	value[1] = value[0];
}



