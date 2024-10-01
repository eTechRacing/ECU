
/*
 * can_comunication.c
 *
 *  Created on: Feb 10, 2024
 *      Author: Carmen Unió Cruz
 *
 *  This Source file contains functions used for the CAN Communication in the
 *  code for the Master BMS.
 *
 *  The function that start with "message_cantx" are the ones in charge of taking the various
 *  signals and variables and preparing the whole message and header for the CAN communication.
 *
 *  The functions that start with "message_canrx" are the ones in charge of taking the data recieved
 *   and adapting it to the thing needed
 */


#include "main.h"
#include "stdlib.h"
#include "can_comunication.h"

uint32_t TxMailbox;
		/*
	 * 	Function: filtercanconfig
	 * 	Purpose: Define the configurations needed to recieve data.
	 * 	Inputs: None. Just states the configurations.
	 */





void CAN1_FILTER_INIT(CAN_FilterTypeDef *canfil){
	   canfil->FilterBank = 0;								// This refers to which filter is being configured. On this case is the filter number 0
	   canfil->FilterMode = CAN_FILTERMODE_IDMASK;					// FilterMode: How are we filtering the incoming messages. Only the messages that coincide with the mask and the filter are accepted
	   canfil->FilterFIFOAssignment = CAN_FILTER_FIFO0;			// Defines at which FIFO is this filter being configured to.
	   canfil->FilterIdHigh = 0x0000;						// MSB: Most Significant Bit. When it's in 0, accepts all the messages
	   canfil->FilterIdLow = 0x0000;						// LSB: Least Significant Bit. When it's in 0, accepts all the messages
	   canfil->FilterMaskIdHigh = 0x0000;					// Most Significant Bit of the mask. When it's in 0, accepts all the messages
	   canfil->FilterMaskIdLow = 0x0000;					// Least Significant Bit of the mask. When it's in 0, accepts all the messages
	   canfil->FilterScale = CAN_FILTERSCALE_32BIT;				// Defines the Filter Scale. (use the 32 bits)
	   canfil->FilterActivation = ENABLE;					// This activates the filter as it is enable
	   canfil->SlaveStartFilterBank = 14;					// Indicates the first filter slave number. In this case it is the principal filter.
}

void CAN2_FILTER_INIT(CAN_FilterTypeDef *canfil){
	   canfil->FilterBank = 14;								// This refers to which filter is being configured. On this case is the filter number 0
	   canfil->FilterMode = CAN_FILTERMODE_IDMASK;					// FilterMode: How are we filtering the incoming messages. Only the messages that coincide with the mask and the filter are accepted
	   canfil->FilterFIFOAssignment = CAN_FILTER_FIFO1;			// Defines at which FIFO is this filter being configured to.
	   canfil->FilterIdHigh = 0x0000;						// MSB: Most Significant Bit. When it's in 0, accepts all the messages
	   canfil->FilterIdLow = 0x0000;						// LSB: Least Significant Bit. When it's in 0, accepts all the messages
	   canfil->FilterMaskIdHigh = 0x0000;					// Most Significant Bit of the mask. When it's in 0, accepts all the messages
	   canfil->FilterMaskIdLow = 0x0000;					// Least Significant Bit of the mask. When it's in 0, accepts all the messages
	   canfil->FilterScale = CAN_FILTERSCALE_32BIT;				// Defines the Filter Scale. (use the 32 bits)
	   canfil->FilterActivation = ENABLE;					// This activates the filter as it is enable
	   canfil->SlaveStartFilterBank = 14;					// Indicates the first filter slave number. In this case it is the principal filter.
}

void CAN1_CONFIG_INIT(CAN_FilterTypeDef canfil, CAN_HandleTypeDef hcan){
    if(HAL_CAN_ConfigFilter(&hcan, &canfil) != HAL_OK) Error_Handler();
    if(HAL_CAN_Start(&hcan) != HAL_OK)Error_Handler();
    if(HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)Error_Handler();
}

void CAN2_CONFIG_INIT(CAN_FilterTypeDef canfil, CAN_HandleTypeDef hcan){
    if(HAL_CAN_ConfigFilter(&hcan, &canfil) != HAL_OK) Error_Handler();
    if(HAL_CAN_Start(&hcan) != HAL_OK)Error_Handler();
    if(HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)Error_Handler();
}



	void message_cantx_SF(CAN_HandleTypeDef hcan, uint8_t UT_FLAG, uint8_t OT_FLAG, uint8_t OV_FLAG, uint8_t UV_FLAG, uint8_t OC_FLAG, uint8_t PRECHARGE_RELAY, uint8_t AIRPLUS, uint8_t AIRMINUS, uint8_t ACCU_INTERLOCK,uint8_t BMS_OK, uint8_t SLAVE_DISC, uint8_t CURRENT_DISC, uint8_t SHOULD_CHARGE, uint8_t IS_CHARGING, uint8_t *TxData, CAN_TxHeaderTypeDef TxHeader){
		uint8_t missatge1 = combined_bits(UT_FLAG, UV_FLAG, OT_FLAG, OV_FLAG, UV_FLAG, OC_FLAG, PRECHARGE_RELAY, AIRPLUS);
		uint8_t missatge2 = combined_bits(AIRMINUS, ACCU_INTERLOCK, BMS_OK, SLAVE_DISC, CURRENT_DISC, SHOULD_CHARGE, IS_CHARGING, 0);
		TxHeader.DLC = 2;							//Number of bites to be transmitted max- 8. DLC: Data Length Code
		TxHeader.IDE = 0;									//IDE: Identifier Extension. ID_STD: Standard Identifier. Dominant(0) = 11 bit ID, Recessive(1) = 29 bit ID
		TxHeader.RTR = 0;									//RTR: Remote Transmission Request, Dominant(0) = Data frame, Recessive (1) = Remote Frame. Type of trace
		TxHeader.StdId = SF;								//Standard identifier ID
		TxHeader.TransmitGlobalTime = DISABLE;				//A temporal mark in the CAN message is not added

		TxData[0] = missatge1;
		TxData[1] = missatge2;
		if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK);
	}

	void message_cantx_AIR_State(CAN_HandleTypeDef hcan1, uint8_t *AIRs_State, uint8_t *TxData, CAN_TxHeaderTypeDef TxHeader){
		TxHeader.DLC = 1; 									//Number of bites to be transmitted max- 8. DLC: Data Length Code
		TxHeader.IDE = 0;									//IDE: Identifier Extension. ID_STD: Standard Identifier. Dominant(0) = 11 bit ID, Recessive(1) = 29 bit ID
		TxHeader.RTR = 0;									//RTR: Remote Transmission Request, Dominant(0) = Data frame, Recessive (1) = Remote Frame. Type of trace
		TxHeader.StdId = 0x91;								//Standard identifier ID
		TxHeader.TransmitGlobalTime = DISABLE;				//A temporal mark in the CAN message is not added
		TxData[0] = AIRs_State[0];
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK);
						//Sent data. The TxData is the buffer where the data is saved//Sent data. The TxData is the buffer where the data is saved
		}

	/*
	 * 	Function: message_cantx_Temperature_State
	 * 	Purpose: Take the inputs and variables needed and send it via CAN.
	 * 	Inputs: Lowest, Highest and Average Temperatures
	 */

	void message_cantx_Temperature_State(CAN_HandleTypeDef hcan1, uint16_t Lowest_CellTemperature, uint16_t Highest_CellTemperature, uint16_t Average_CellTemperature, uint8_t *TxData, CAN_TxHeaderTypeDef TxHeader){

		TxHeader.DLC = 6; 									//Number of bites to be transmitted max- 8. DLC: Data Length Code
		TxHeader.IDE = 0;									//IDE: Identifier Extension. ID_STD: Standard Identifier. Dominant(0) = 11 bit ID, Recessive(1) = 29 bit ID
		TxHeader.RTR = 0;									//RTR: Remote Transmission Request, Dominant(0) = Data frame, Recessive (1) = Remote Frame. Type of trace
		TxHeader.StdId = 0x93;								//Standard identifier ID
		TxHeader.TransmitGlobalTime = DISABLE;				//A temporal mark in the CAN message is not added
		TxData[0] = Lowest_CellTemperature >> 8;
		TxData[1] = Lowest_CellTemperature;
		TxData[2] = Highest_CellTemperature >> 8;
		TxData[3] = Highest_CellTemperature;
		TxData[4] = Average_CellTemperature >> 8;
		TxData[5] = Average_CellTemperature;
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK);
		}

	/*
	 * 	Function: message_cantx_Charger_Parameters
	 * 	Purpose: Take the inputs and variables needed and send it via CAN.
	 * 	Inputs: Maximum Voltage and Current
	 */

	void message_cantx_Charger_Parameters(CAN_HandleTypeDef hcan1, uint16_t Charger_MaxVoltage, uint8_t Charger_MaxCurrent, uint8_t *TxData, CAN_TxHeaderTypeDef TxHeader){

		TxHeader.DLC = 3; 									//Number of bites to be transmitted max- 8. DLC: Data Length Code
		TxHeader.IDE = 0;									//IDE: Identifier Extension. ID_STD: Standard Identifier. Dominant(0) = 11 bit ID, Recessive(1) = 29 bit ID
		TxHeader.RTR = 0;									//RTR: Remote Transmission Request, Dominant(0) = Data frame, Recessive (1) = Remote Frame. Type of trace
		TxHeader.StdId = 0x9D;								//Standard identifier ID
		TxHeader.TransmitGlobalTime = DISABLE;				//A temporal mark in the CAN message is not added
		TxData[0] = Charger_MaxVoltage;						//Sent data. The TxData is the buffer where the data is saved
		TxData[1] = Charger_MaxVoltage>>8;
		TxData[2] = Charger_MaxCurrent;
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK);
						//Sent data. The TxData is the buffer where the data is saved
		}

	/*
	 * 	Function: message_cantx_Keep_Alive
	 * 	Purpose: Take the inputs and variables needed and send it via CAN.
	 * 	Inputs: Keep Alive
	 */

	void message_cantx_Keep_Alive(CAN_HandleTypeDef hcan1, uint8_t Keep_Alive, uint8_t *TxData, CAN_TxHeaderTypeDef TxHeader){

		TxHeader.DLC = 1; 									//Number of bites to be transmitted max- 8. DLC: Data Length Code
		TxHeader.IDE = 0;									//IDE: Identifier Extension. ID_STD: Standard Identifier. Dominant(0) = 11 bit ID, Recessive(1) = 29 bit ID
		TxHeader.RTR = 0;									//RTR: Remote Transmission Request, Dominant(0) = Data frame, Recessive (1) = Remote Frame. Type of trace
		TxHeader.StdId = 0xCA;								//Standard identifier ID
		TxHeader.TransmitGlobalTime = DISABLE;				//A temporal mark in the CAN message is not added
		TxData[0] = Keep_Alive;
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK);
				//Sent data. The TxData is the buffer where the data is saved
		}

	/*
	 * 	Function: message_cantx_Shutdown
	 * 	Purpose: Take the inputs and variables needed and send it via CAN.
	 * 	Inputs: Package Interlock, BMS (BMS_OK) and IMD -> Bits
	 */

	void message_cantx_Shutdown(CAN_HandleTypeDef hcan1, unsigned char Shutdown_PackageIntck, unsigned char Shutdown_BMS, unsigned char Shutdown_IMD, uint8_t *TxData, CAN_TxHeaderTypeDef TxHeader){

		TxHeader.DLC = 1; 									//Number of bites to be transmitted max- 8. DLC: Data Length Code
		TxHeader.IDE = 0;									//IDE: Identifier Extension. ID_STD: Standard Identifier. Dominant(0) = 11 bit ID, Recessive(1) = 29 bit ID
		TxHeader.RTR = 0;									//RTR: Remote Transmission Request, Dominant(0) = Data frame, Recessive (1) = Remote Frame. Type of trace
		TxHeader.StdId = 0x90;								//Standard identifier ID
		TxHeader.TransmitGlobalTime = DISABLE;				//A temporal mark in the CAN message is not added
		TxData[0] = (Shutdown_IMD << 2) | (Shutdown_BMS << 1) | Shutdown_PackageIntck; 	//Sent data. The TxData is the buffer where the data is saved
					// Lectura de pinaje | Shutdown_BMS = BMS_OK | Lectura de pinaje
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK);
						//Sent data. The TxData is the buffer where the data is saved
	}

	/*
	 * 	Function: message_cantx_SoC_SoH
	 * 	Purpose: Take the inputs and variables needed and send it via CAN.
	 * 	Inputs: State of Charge and Health
	 */

	void message_cantx_SoC_SoH(CAN_HandleTypeDef hcan1, uint16_t SoC_Avg, uint16_t SoC_High, uint16_t SoC_Low, uint16_t SoH, uint8_t *TxData, CAN_TxHeaderTypeDef TxHeader){

		TxHeader.DLC = 8; 									//Number of bites to be transmitted max- 8. DLC: Data Length Code
		TxHeader.IDE = 0;									//IDE: Identifier Extension. ID_STD: Standard Identifier. Dominant(0) = 11 bit ID, Recessive(1) = 29 bit ID
		TxHeader.RTR = 0;									//RTR: Remote Transmission Request, Dominant(0) = Data frame, Recessive (1) = Remote Frame. Type of trace
		TxHeader.StdId = 0x95;								//Standard identifier ID
		TxHeader.TransmitGlobalTime = DISABLE;				//A temporal mark in the CAN message is not added
		TxData[0] = SoC_Avg;								//Sent data. The TxData is the buffer where the data is saved
		TxData[1] = SoC_Avg >> 8;							//Sent data. The TxData is the buffer where the data is saved
		TxData[2] = SoC_High;								//Sent data. The TxData is the buffer where the data is saved
		TxData[3] = SoC_High >> 8;							//Sent data. The TxData is the buffer where the data is saved
		TxData[4] = SoC_Low;								//Sent data. The TxData is the buffer where the data is saved
		TxData[5] = SoC_Low >> 8;							//Sent data. The TxData is the buffer where the data is saved
		TxData[6] = SoH;									//Sent data. The TxData is the buffer where the data is saved
		TxData[7] = SoH >> 8;
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK);
				//Sent data. The TxData is the buffer where the data is saved
		}

	/*
	 * 	Function: message_cantx_SoC_SoH
	 * 	Purpose: Takes different bits and puts them all together in a variable called byte.
	 * 	Inputs: The different bits in order from the MSB (bit 8) to the LSB (bit 1)
	 */

	uint8_t combined_bits(unsigned char bit1, unsigned char bit2, unsigned char bit3, unsigned char bit4,
						   unsigned char bit5, unsigned char bit6, unsigned char bit7, unsigned char bit8){

		uint8_t byte = (uint8_t)(bit8 << 7) | (uint8_t)(bit7 << 6) | (uint8_t)(bit6 << 5) | (uint8_t)(bit5 << 4) |
		                (uint8_t)(bit4 << 3) | (uint8_t)(bit3 << 2) | (uint8_t)(bit2 << 1) | (uint8_t)bit1;

		// Bit 1 is the Least Significant Bit and it goes up until Bit 8 which has the Most Significant Bit
		// Byte -> [Bit 8  Bit 7  Bit 6  Bit 5  Bit 4  Bit 3  Bit 2  Bit 1]

		return byte;
	}

	/*
	 * 	Function: message_cantx_Voltage_State
	 * 	Purpose: Take the inputs and variables needed and send it via CAN.
	 * 	Inputs: Lowest, Lowest and Average Cell Voltage
	 */

	void message_cantx_Voltage_State(CAN_HandleTypeDef hcan1,uint16_t Lowest_CellVoltage, uint16_t Highest_CellVoltage, uint32_t Sum_Of_All_Voltages,uint8_t *TxData, CAN_TxHeaderTypeDef TxHeader){

		TxHeader.DLC = 7; 									//Number of bites to be transmitted max- 8. DLC: Data Length Code
		TxHeader.IDE = 0;									//IDE: Identifier Extension. ID_STD: Standard Identifier. Dominant(0) = 11 bit ID, Recessive(1) = 29 bit ID
		TxHeader.RTR = 0;									//RTR: Remote Transmission Request, Dominant(0) = Data frame, Recessive (1) = Remote Frame. Type of trace
		TxHeader.StdId = 0x92;								//Standard identifier ID
		TxHeader.TransmitGlobalTime = DISABLE;				//A temporal mark in the CAN message is not added
		TxData[0] = Lowest_CellVoltage >> 8;						//Sent data. The TxData is the buffer where the data is saved
		TxData[1] = Lowest_CellVoltage;
		TxData[2] = Highest_CellVoltage >> 8;
		TxData[3] = Highest_CellVoltage;
		TxData[4] = Sum_Of_All_Voltages >> 16;
		TxData[5] = Sum_Of_All_Voltages>> 8;
		TxData[6] = Sum_Of_All_Voltages;
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK);
						//Sent data. The TxData is the buffer where the data is saved

		}


	/*
	 * 	Function:  message_canrx_AIRs_Request
	 * 	Purpose: Takes the Data form the CAN Message and, depending on the result, sets one state of the relays or another.
	 * 	Inputs: Pins of the Relays and their different ports.
	 */


	void CANRX(uint8_t *RxData, CAN_RxHeaderTypeDef RxHeader, unsigned char *Keep_Alive){
		switch (RxHeader.IDE){
		case 0x82://AIRs Request
			message_canrx_AIRs_Request(RxData);
			break;
		case 0x3C2://ACCU Current
			break;
		case 0x9A://Sync Charger
			message_canrx_Syncronism(RxData, Keep_Alive);
			break;
		}
	}

	/*
	 * 	Function: message_canrx_Current_Sensor
	 * 	Purpose: Takes the Data form the CAN Message and, sorts it in different
	 * 	Inputs: Pins of the Relays and their different ports.
	 */

//	void message_canrx_Current_Sensor(uint8_t RxData[5], unsigned char *Current_Error, uint32_t *Current_Value) {
//	    Current_Value = ((uint32_t)RxData[0] << 24) | ((uint32_t)RxData[1] << 16) | ((uint32_t)RxData[2] << 8) | RxData[3]; 		// Move all bytes to a variable
//	    Current_Error = RxData[4] & 0x01; 																							// Get the value of the error 1-> Error   0-> No error
//	}

	/*
	 * 	Function: uncombined_bytes
	 * 	Purpose: Takes a byte of data and separes each bit in different variables
	 * 	Inputs: A byte of data
	 */

	unsigned char uncombined_bytes(uint8_t Byte, unsigned char *byte_array){

			byte_array[0] = (Byte >> 7) & 0x01;
		    byte_array[1] = (Byte >> 6) & 0x01;
		    byte_array[2] = (Byte >> 5) & 0x01;
		    byte_array[3] = (Byte >> 4) & 0x01;
		    byte_array[4] = (Byte >> 3) & 0x01;
		    byte_array[5] = (Byte >> 2) & 0x01;
		    byte_array[6] = (Byte >> 1) & 0x01;
		    byte_array[7] = Byte & 0x01;

		    return *byte_array;

		}

	/*
	 * 	Function: message_canrx_Syncronism
	 * 	Purpose: Recieves the Syncronism signal, and increases the counter of Keep Alive until it reaches 255.
	 * 	Inputs: Signal of the Syncronism
	 */

	void message_canrx_Syncronism(uint8_t *RxData, unsigned char *Keep_Alive){
		if (RxData[0] != 255 ){
			Keep_Alive++;
		} else {
			Keep_Alive = 0;
		}
	}











	void message_cantx_FECU_Data1(CAN_HandleTypeDef hcan2,uint16_t APPS1, uint16_t APPS2, uint16_t Brake_Pedal,uint16_t Steering_Sensor,uint8_t *TxData, CAN_TxHeaderTypeDef TxHeader){

		TxHeader.DLC = 8; 									//Number of bites to be transmitted max- 8. DLC: Data Length Code
		TxHeader.IDE = 0;									//IDE: Identifier Extension. ID_STD: Standard Identifier. Dominant(0) = 11 bit ID, Recessive(1) = 29 bit ID
		TxHeader.RTR = 0;									//RTR: Remote Transmission Request, Dominant(0) = Data frame, Recessive (1) = Remote Frame. Type of trace
		TxHeader.StdId = 0x90;								//Standard identifier ID
		TxHeader.TransmitGlobalTime = DISABLE;				//A temporal mark in the CAN message is not added
		TxData[0] = APPS1 >> 8;						//Sent data. The TxData is the buffer where the data is saved
		TxData[1] = APPS1;
		TxData[2] = APPS2 >> 8;
		TxData[3] = APPS2;
		TxData[4] = Brake_Pedal >> 8;						//Sent data. The TxData is the buffer where the data is saved
		TxData[5] = Brake_Pedal;
		TxData[6] = Steering_Sensor >> 8;
		TxData[7] = Steering_Sensor;
		if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK);
						//Sent data. The TxData is the buffer where the data is saved

		}

	void message_cantx_FECU_Data2(CAN_HandleTypeDef hcan2,uint16_t Susp_F_R, uint16_t Susp_F_L, uint32_t Pitot_Sensor, uint8_t *TxData, CAN_TxHeaderTypeDef TxHeader){

		TxHeader.DLC = 6; 									//Number of bites to be transmitted max- 8. DLC: Data Length Code
		TxHeader.IDE = 0;									//IDE: Identifier Extension. ID_STD: Standard Identifier. Dominant(0) = 11 bit ID, Recessive(1) = 29 bit ID
		TxHeader.RTR = 0;									//RTR: Remote Transmission Request, Dominant(0) = Data frame, Recessive (1) = Remote Frame. Type of trace
		TxHeader.StdId = 0x91;								//Standard identifier ID
		TxHeader.TransmitGlobalTime = DISABLE;				//A temporal mark in the CAN message is not added
		TxData[0] = Susp_F_R >> 8;						//Sent data. The TxData is the buffer where the data is saved
		TxData[1] = Susp_F_R;
		TxData[2] = Susp_F_L >> 8;
		TxData[3] = Susp_F_L;
		TxData[4] = Pitot_Sensor>> 8;
		TxData[5] = Pitot_Sensor;
		if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK);
						//Sent data. The TxData is the buffer where the data is saved

		}

	void message_cantx_FECU_SDC(CAN_HandleTypeDef hcan,uint8_t SDC_Setas, uint8_t SDC_BSPD_Intertia, uint8_t SDC_BOTS,uint8_t *TxData, CAN_TxHeaderTypeDef TxHeader){

		TxHeader.DLC = 1; 									//Number of bites to be transmitted max- 8. DLC: Data Length Code
		TxHeader.IDE = 0;									//IDE: Identifier Extension. ID_STD: Standard Identifier. Dominant(0) = 11 bit ID, Recessive(1) = 29 bit ID
		TxHeader.RTR = 0;									//RTR: Remote Transmission Request, Dominant(0) = Data frame, Recessive (1) = Remote Frame. Type of trace
		TxHeader.StdId = 0xD0;								//Standard identifier ID
		TxHeader.TransmitGlobalTime = DISABLE;				//A temporal mark in the CAN message is not added
		TxData[0] = (0b00000111)&(0b00000000|SDC_Setas|SDC_BSPD_Intertia<<1|SDC_BOTS<<2);						//Sent data. The TxData is the buffer where the data is saved
		if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK);
						//Sent data. The TxData is the buffer where the data is saved

		}

	void message_cantx_FECU_Keep_Alive(CAN_HandleTypeDef hcan, uint8_t Keep_Alive,uint8_t *TxData, CAN_TxHeaderTypeDef TxHeader){

		TxHeader.DLC = 1; 									//Number of bites to be transmitted max- 8. DLC: Data Length Code
		TxHeader.IDE = 0;									//IDE: Identifier Extension. ID_STD: Standard Identifier. Dominant(0) = 11 bit ID, Recessive(1) = 29 bit ID
		TxHeader.RTR = 0;									//RTR: Remote Transmission Request, Dominant(0) = Data frame, Recessive (1) = Remote Frame. Type of trace
		TxHeader.StdId = 0xCA;								//Standard identifier ID
		TxHeader.TransmitGlobalTime = DISABLE;				//A temporal mark in the CAN message is not added
		TxData[0] = Keep_Alive;						//Sent data. The TxData is the buffer where the data is saved
		if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK);
						//Sent data. The TxData is the buffer where the data is saved

		}


	void message_cantx_RECU_Data(CAN_HandleTypeDef hcan2,uint16_t Susp_R_R, uint16_t Susp_R_L,uint8_t *TxData, CAN_TxHeaderTypeDef TxHeader){

		TxHeader.DLC = 4; 									//Number of bites to be transmitted max- 8. DLC: Data Length Code
		TxHeader.IDE = 0;									//IDE: Identifier Extension. ID_STD: Standard Identifier. Dominant(0) = 11 bit ID, Recessive(1) = 29 bit ID
		TxHeader.RTR = 0;									//RTR: Remote Transmission Request, Dominant(0) = Data frame, Recessive (1) = Remote Frame. Type of trace
		TxHeader.StdId = 0xA0;								//Standard identifier ID
		TxHeader.TransmitGlobalTime = DISABLE;				//A temporal mark in the CAN message is not added
		TxData[0] = Susp_R_R >> 8;						//Sent data. The TxData is the buffer where the data is saved
		TxData[1] = Susp_R_R;
		TxData[2] = Susp_R_L >> 8;
		TxData[3] = Susp_R_L;
		if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK);

		}

	void message_cantx_RECU_SDC(CAN_HandleTypeDef hcan,uint8_t TSMS_TSMP, uint8_t Right_TS, uint8_t Left_TS, uint8_t HVBox, uint8_t HVD,uint8_t *TxData, CAN_TxHeaderTypeDef TxHeader){

			TxHeader.DLC = 1; 									//Number of bites to be transmitted max- 8. DLC: Data Length Code
			TxHeader.IDE = 0;									//IDE: Identifier Extension. ID_STD: Standard Identifier. Dominant(0) = 11 bit ID, Recessive(1) = 29 bit ID
			TxHeader.RTR = 0;									//RTR: Remote Transmission Request, Dominant(0) = Data frame, Recessive (1) = Remote Frame. Type of trace
			TxHeader.StdId = 0xD1;								//Standard identifier ID
			TxHeader.TransmitGlobalTime = DISABLE;				//A temporal mark in the CAN message is not added
			TxData[0] = (0b00011111)&(0b00000000|TSMS_TSMP|Right_TS<<1|Left_TS<<2|HVBox<<3|HVD<<4);						//Sent data. The TxData is the buffer where the data is saved
			if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK);

			}

	void message_cantx_RECU_Keep_Alive(CAN_HandleTypeDef hcan, uint8_t Keep_Alive,uint8_t *TxData, CAN_TxHeaderTypeDef TxHeader){

		TxHeader.DLC = 1; 									//Number of bites to be transmitted max- 8. DLC: Data Length Code
		TxHeader.IDE = 0;									//IDE: Identifier Extension. ID_STD: Standard Identifier. Dominant(0) = 11 bit ID, Recessive(1) = 29 bit ID
		TxHeader.RTR = 0;									//RTR: Remote Transmission Request, Dominant(0) = Data frame, Recessive (1) = Remote Frame. Type of trace
		TxHeader.StdId = 0xCB;								//Standard identifier ID
		TxHeader.TransmitGlobalTime = DISABLE;				//A temporal mark in the CAN message is not added
		TxData[0] = Keep_Alive;						//Sent data. The TxData is the buffer where the data is saved
		if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK);

		}

