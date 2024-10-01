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

void filtreLP_init(filtreLP_struct *v)
{
    float aux;
    aux = v->fc* v->Ts;                                  // fc*Ts (fc in kHz)
    aux = aux * 1000.0;                                  // Convert kHz to Hz
    aux = PI2 * aux;                                      // 2*pi*fc*Ts
    v->alfa = aux/(aux + 1.0);                            // 2*pi*fc*Ts/(2*pi*fc*Ts + 1)
}

/**
 * @brief Calculates the first-order filter.
 *
 * @param v Pointer to the first-order filter structure.
 *
 * @note This function calculates the first-order filter.
 */
void filtreLP_calc(filtreLP_struct *v)
{
    if (v->enable)
    {
        v->out = (v->alfa * (v->in - v->out)) + v->out;    // Filter out(k) = alfa*in(k) + (1-alfa)*out(k-1)
    }
    else
    {
        v->out = v->in;        // Without filter. Out(k) = In(k)
    }
}

float applyLowPassFilter(float input, filtreLP_struct *filter)
{
    filter->in = input;     // Update the input value
    filtreLP_calc(filter);  // Apply the filter
    return filter->out;     // Return the filtered output
}



