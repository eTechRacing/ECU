/*
 * CAN.c
 *
 *  Created on: Feb 19, 2025
 *      Author: Pol
 */

#include "CAN.h"

void Init_CAN_Filter(CAN_HandleTypeDef *hcan1){
    CAN_FilterTypeDef canfil1;
    canfil1.FilterBank = 0;								// This refers to which filter is being configured. On this case is the filter number 0
    canfil1.FilterMode = CAN_FILTERMODE_IDMASK;					// FilterMode: How are we filtering the incoming messages. Only the messages that coincide with the mask and the filter are accepted
    canfil1.FilterFIFOAssignment = CAN_RX_FIFO0;			// Defines at which FIFO is this filter being configured to.
    canfil1.FilterIdHigh = 0x0000;						// MSB: Most Significant Bit. When it's in 0, accepts all the messages
    canfil1.FilterIdLow = 0x0000;						// LSB: Least Significant Bit. When it's in 0, accepts all the messages
    canfil1.FilterMaskIdHigh = 0x0000;					// Most Significant Bit of the mask. When it's in 0, accepts all the messages
    canfil1.FilterMaskIdLow = 0x0000;					// Least Significant Bit of the mask. When it's in 0, accepts all the messages
    canfil1.FilterScale = CAN_FILTERSCALE_32BIT;				// Defines the Filter Scale. (use the 32 bits)
    canfil1.FilterActivation = ENABLE;					// This activates the filter as it is enable
    canfil1.SlaveStartFilterBank = 13;					// Indicates the first filter slave number. In this case it is the principal filter.

    HAL_CAN_ConfigFilter(hcan1, &canfil1);
}

