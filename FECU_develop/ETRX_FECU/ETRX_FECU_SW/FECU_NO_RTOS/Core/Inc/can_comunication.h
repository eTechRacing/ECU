/*
 * can_comunication.h
 *
 *  Created on: Feb 10, 2024
 *      Author: Carmen Unió Cruz
 *
 *  This Header file contains the defined functions used for the
 *  Can Communication in the code for the Master BMS.
 */

#ifndef INC_CAN_COMUNICATION_H_
#define INC_CAN_COMUNICATION_H_

#include "main.h"

//COBIDS

#define DISPLAY_SYNC 0xE0
#define ETAS_SYNC 0x80
#define AIRS_REQUEST 0x82



void filtercanconfig(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef canfil, int CAN_FILTERMODE, int CAN_RX_FIFO, int CAN_FILTERSCALE);
void message_cantx_AIR_State(char AIRs_State, uint8_t *TxData, CAN_TxHeaderTypeDef TxHeader);
void message_cantx_Temperature_State(CAN_HandleTypeDef hcan2, uint16_t Lowest_CellTemperature, uint16_t Highest_CellTemperature, uint16_t Average_CellTemperature, uint8_t *TxData, CAN_TxHeaderTypeDef TxHeader);
void message_cantx_Charger_Parameters(uint16_t Charger_MaxVoltage, uint8_t Charger_MaxCurrent, uint8_t *TxData, CAN_TxHeaderTypeDef TxHeader);
void message_cantx_Keep_Alive(CAN_HandleTypeDef hcan2, uint8_t Keep_Alive, uint8_t *TxData, CAN_TxHeaderTypeDef TxHeader);
void message_cantx_Shutdown(unsigned char Shutdown_PackageIntck, unsigned char Shutdown_BMS, unsigned char Shutdown_IMD, uint8_t *TxData, CAN_TxHeaderTypeDef TxHeader);
void message_cantx_SoC_SoH(uint16_t SoC_Avg, uint16_t SoC_High, uint16_t SoC_Low, uint16_t SoH, uint8_t *TxData, CAN_TxHeaderTypeDef TxHeader);
uint8_t combined_bits(unsigned char bit1, unsigned char bit2, unsigned char bit3, unsigned char bit4, unsigned char bit5, unsigned char bit6, unsigned char bit7, unsigned char bit8);
void message_cantx_Voltage_State(CAN_HandleTypeDef hcan2,uint16_t Lowest_CellVoltage, uint16_t Highest_CellVoltage, uint32_t Sum_Of_All_Voltages,uint8_t *TxData, CAN_TxHeaderTypeDef TxHeader);
void message_canrx_AIRs_Request(uint8_t RxData);
void message_canrx_Current_Sensor(uint8_t RxData[5], unsigned char *Current_Error, uint32_t *Current_Value);
unsigned char uncombined_bytes(uint8_t Byte, unsigned char *byte_array);
void message_canrx_Syncronism(uint8_t RxData[1], unsigned char *Keep_Alive);
void message_cantx_FECU_Data1(CAN_HandleTypeDef hcan2,uint16_t APPS1, uint16_t APPS2, uint16_t Brake_Pedal,uint16_t Steering_Sensor,uint8_t *TxData, CAN_TxHeaderTypeDef TxHeader);
void message_cantx_FECU_DATA2(CAN_HandleTypeDef hcan2,uint16_t Susp_F_R, uint16_t Susp_F_L, uint32_t Pitot_Sensor, uint8_t *TxData, CAN_TxHeaderTypeDef TxHeader);
void message_cantx_FECU_SDC(CAN_HandleTypeDef hcan2,uint8_t SDC_Setas, uint8_t SDC_BSPD_Intertia, uint8_t SDC_BOTS,uint8_t *TxData, CAN_TxHeaderTypeDef TxHeader);
void message_cantx_FECU_Keep_Alive(CAN_HandleTypeDef hcan2, uint8_t Keep_Alive, uint8_t *TxData, CAN_TxHeaderTypeDef TxHeader);



#endif /* INC_CAN_COMUNICATION_H_ */
