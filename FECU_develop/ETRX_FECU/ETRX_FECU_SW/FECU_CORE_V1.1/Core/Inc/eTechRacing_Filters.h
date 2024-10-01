/*
 * eTechRacing_Filters.h
 *
 *  Created on: May 24, 2024
 *      Author: Andreu
 */

#ifndef INC_ETECHRACING_FILTERS_H_
#define INC_ETECHRACING_FILTERS_H_



#endif /* INC_ETECHRACING_FILTERS_H_ */


/**
 * @brief First-order low-pass filter.
 *
 */
#include <math.h>
#include <stdio.h>
#include "stdint.h"


#define PI2 (2.0 * M_PI)  // Define PI2 as 2*pi

//typedef struct
//{
//        float                 in;                                /**< Input signal */
//        float                 out;                        /**< Output signal */
//        float                 alfa;                        /**< Filter coefficient */
//        float                Ts;                                /**< Execution period */
//        float                fc;                                /**< Cutoff frequency */
//        unsigned char        enable;                        /**< Enable flag */                            /*< Pointer to the calculation function */
//} filtreLP_struct;
//
///**
// * @brief Initializes the first-order low-pass filter.
// * @param v Pointer to the filter structure.
// */
//void filtreLP_init( filtreLP_struct *v);
//
///**
// * @brief Calculates the output of the first-order low-pass filter.
// * @param v Pointer to the filter structure.
// */
//void filtreLP_calc( filtreLP_struct *v);

void applyLowPassFilter(uint16_t *value);
