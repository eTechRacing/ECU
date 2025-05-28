/*
 * 	etr_carstate.c
 *
 *  Created on: Feb 25, 2025
 *      Author: cunio
 */
#include <stdio.h>
#include <stdint.h>
#include "DASH/etr_carstate.h"
#include "DASH/buttons.h"
#include "DASH/etr_carstate.h"
#include "CAN/CAN_X_2025.h"


uint8_t RacingMode_Send;
uint8_t CoolingRequest;
uint8_t selectedBox;


const int NUM_SCREENS_PER_STATE[] = {
    5,  // DASH_0_ETR
    5,  // DASH_3_PRECHARGE
    1,  // DASH_6_PRECHARGE_STATUS
    1,  // DASH_9_PRECHARGE_FINISHED
    11, // DASH_12_RACING_MENU
    1,  // DASH_14_INVERTERS
    5,  // DASH_15_RACING_MODE
    1   // DASH_21_ERROR
};

DASH_State Screen = {DASH_0_ETR, DASH_0_ETR, SCREEN_1, SCREEN_1};

void resetAllSignals(void) {
    New_Signal_193 = 0;
    el_AUTO_STATUS = 0;
    Inv_R_Iq = 0;
    Inv_R_Icommand = 0;
    Inv_R_Iactual = 0;
    Inv_L_Iq = 0;
    Inv_L_Icommand = 0;
    Inv_L_Iactual = 0;
    SOE = 0;
    ETAS_MSG_Counter = 0;
    BMS_Alive = 0;
    BMS_Balancing_Enable = 0;
    BMS_Charge_Flag = 0;
    BMS_CAN_Disconnection = 0;
    BMS_Voltage_Disconnection = 0;
    BMS_NTC_Disconnection = 0;
    BMS_Current_Disconnection = 0;
    BMS_Undertemperature = 0;
    BMS_Overtemperature = 0;
    BMS_Overvoltage = 0;
    BMS_Undervoltage = 0;
    Precharge_State = 0;
    AIR_Plus_State = 0;
    AIR_Minus_State = 0;
    Shutdown_PackageIntck = 0;
    Charger_Accumulator_Current = 0;
    Charger_Lowest_Cell_Temperature = 0;
    Charger_Highest_Cell_Temperature = 0;
    Charger_Average_Cell_Temperature = 0;
    Charger_Acumulator_Voltage = 0;
    Charger_Highest_Cell_Voltage = 0;
    Charger_Lowest_Cell_Voltage = 0;
    Charger_PrechargeOK = 0;
    Charger_AIRs_State = 0;
    Charger_AIRs_Request = 0;
    Charger_Sync = 0;
    VDC_Steering_Deadzone = 0;
    VDC_Min_Tyre_Slip = 0;
    VDC_Max_Tyre_Slip = 0;
    VDC_Max_TV_DiffTq = 0;
    VDC_Max_Steering_Angle = 0;
    VDC_AP_SatUp = 0;
    VDC_AP_SatDown = 0;
    TotalTime = 0;
    LapCount = 0;
    LapTime = 0;
    Average_CellTemp = 0;
    Accumulator_Voltage = 0;
    Accumulator_Current = 0;
    Shutdown_IMD = 0;
    Shutdown_BMS = 0;
    AIRs_State = 0;
    Highest_CellVoltage = 0;
    Highest_CellTemp = 0;
    Lowest_CellVoltage = 0;
    Lowest_CellTemp = 0;
    DASH_LV = 0;
    DASH_5V = 0;
    DASH_3V3 = 0;
    Dash_Alive = 0;
    IMD_OK = 0;
    BMS_OK = 0;
    BMS_LV = 0;
    BMS_5V = 0;
    BMS_12V = 0;
    Precharge_Percentage = 0;
    Power = 0;
    SOC_High = 0;
    SOC_Low = 0;
    SOC_Avg = 0;
    el_Track_Valid = 0;
    el_Slip_Valid = 0;
    el_Curvature_Valid = 0;
    el_SlipAngle = 0;
    el_CurvatureRadius = 0;
    el_AngleTrack = 0;
    el_Vel_GPS_N = 0;
    el_Vel_GPS_E = 0;
    el_Vel_GPS_D = 0;
    InvertersAction = 0;
    Relay_Error = 0;
    Regenerative_Enable = 0;
    TC_Warning = 0;
    Sensorics_Mode = 0;
    el_Vel_OK = 0;
    Torque_OK = 0;
    APPS_Implausibility = 0;
    Disconnection_BrakePressure2 = 0;
    Disconnection_BrakePressure1 = 0;
    Critical_Signal_Disconnection = 0;
    Critical_CAN_Disconnection = 0;
    Disconnection_InvR = 0;
    Disconnection_InvL = 0;
    Disconnection_Susp_R_R = 0;
    Disconnection_Susp_R_L = 0;
    Disconnection_Susp_F_R = 0;
    Disconnection_Susp_F_L = 0;
    Disconnection_SteeringSensor = 0;
    Disconnection_Rear = 0;
    Disconnection_Pitot = 0;
    Disconnection_Front = 0;
    Disconnection_Ellipse = 0;
    Disconnection_DashBoard = 0;
    Disconnection_BrakePedal = 0;
    Disconnection_BMS = 0;
    Disconnection_APPS2 = 0;
    Disconnection_APPS1 = 0;
    Inv_Speed = 0;
    el_VEL = 0;
    SteeringSensor_Value = 0;
    BrakePedal_Value = 0;
    APPS2_Value = 0;
    APPS1_Value = 0;
    Pump_R = 0;
    Pump_L = 0;
    Button_2 = 0;
    Button_1 = 0;
    TV_Level = 0;
    TC_Level = 0;
    Driver = 1;
    RacingMode = 1;
    EnableDrive_Order = 0;

    // CAR STATE VARIABLES
    RacingMode_Send = 0;
    CoolingRequest = 0;
    selectedBox = 0;
}


void init_rules(void){
	HAL_GPIO_WritePin(IMD_LED_GPIO_Port, IMD_LED_Pin, 1);
	HAL_GPIO_WritePin(BMS_LED_GPIO_Port, BMS_LED_Pin, 1);

	HAL_Delay(2500);

	HAL_GPIO_WritePin(IMD_LED_GPIO_Port, IMD_LED_Pin, 0);
	HAL_GPIO_WritePin(BMS_LED_GPIO_Port, BMS_LED_Pin, 0);
}

void refreshGPIOs (void){
	if (Shutdown_PackageIntck == 1 && Disconnection_BMS == 0) {
		HAL_GPIO_WritePin(IMD_LED_GPIO_Port, IMD_LED_Pin, 0);
		HAL_GPIO_WritePin(BMS_LED_GPIO_Port, BMS_LED_Pin, 0);
	}
	if (IMD_OK == 0 || Disconnection_BMS == 1){
		HAL_GPIO_WritePin(IMD_LED_GPIO_Port, IMD_LED_Pin, 1);
		FLAG++;
	}
	if (BMS_OK == 0 || Disconnection_BMS == 1) {
		HAL_GPIO_WritePin(BMS_LED_GPIO_Port, BMS_LED_Pin, 1);
		FLAG++;
	}

	if (Screen.ActualState == DASH_5_INVERTERS){
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);
	}
	if (Screen.ActualState != DASH_5_INVERTERS){
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
	}
}

	// haces la pantalla estatica, y los datos se ponen en la otra funcion
void refreshScreen(void) {

    		// RESETEA LA PANTALLA SI SE CAMBIA EL ESTADO
    if (Screen.PreviousState != Screen.ActualState){
    	Screen.ActualScreen = SCREEN_1;
    	Screen.PreviousState = Screen.ActualState;
    }

    switch(Screen.ActualState) {

        case DASH_0_ETR:

        		/*BUTTON DOWN*/
            if (pendingButtonEvent == EVENT_BUTTON_DOWN) {

            	if (Screen.ActualScreen == SCREEN_5 && CoolingRequest == 1){

            		if (selectedBox == 0 || selectedBox == 1) {
            			selectedBox += 2;

            		} else if (selectedBox == 2 || selectedBox == 3) {
            			selectedBox = 4;

            		} else if (selectedBox == 4) {
            			selectedBox = 0;

            		} else if (Screen.ActualScreen == SCREEN_5){
            			Screen.ActualScreen = SCREEN_1;

            		} else {
            			Screen.ActualScreen ++;

            		}
            	}
            }
            	/*BUTTON UP*/
            if (pendingButtonEvent == EVENT_BUTTON_UP) {

            	if ((Screen.ActualScreen = SCREEN_5) && (CoolingRequest == 1)){
            		if (selectedBox == 4) {
            			selectedBox = 2;

            		} else if (selectedBox == 2 || selectedBox == 3) {
            			selectedBox -= 2;

            		} else if (selectedBox == 0 || selectedBox == 1) {
            			selectedBox = 4;

            		} else if (Screen.ActualScreen == SCREEN_1){
            			Screen.ActualScreen = SCREEN_5;

            		} else {
            			Screen.ActualScreen --;

            		}
            	}
            }

            	/*BUTTON RIGHT*/
            if (pendingButtonEvent == EVENT_BUTTON_RIGHT) {
            	if (Screen.ActualScreen == SCREEN_5 && CoolingRequest == 1){
            	    selectedBox = (selectedBox + 1) % 5;  // Mueve a la derecha
            	}
            }

            	/*BUTTON LEFT*/
            if (pendingButtonEvent == EVENT_BUTTON_LEFT) {
            	if (Screen.ActualScreen == SCREEN_5 && CoolingRequest == 1){
            		if (selectedBox == 0) {
            		    selectedBox = 4;
            		} else {
            		    selectedBox = selectedBox - 1;
            		}
            	}
            }

            	/*BUTTON OK*/
            if (pendingButtonEvent == EVENT_BUTTON_OK) {
            	if (Screen.ActualScreen == SCREEN_5 && CoolingRequest == 0){
            		CoolingRequest = 1;
            		selectedBox = 0;
            	} else {
            		switch (selectedBox) {
                    	case 0: Fans_R = (Fans_R + 1) % 3; break;
                    	case 1: Fans_L = (Fans_L + 1) % 3; break;
                    	case 2: Pump_R = (Pump_R + 1) % 2; break;
                    	case 3: Pump_L = (Pump_L + 1) % 2; break;
                    	case 4: CoolingRequest = 0; break;  			// Salir del modo edición
            		}
            	}
            }

            break;

        case DASH_1_PRECHARGE:

            if (pendingButtonEvent == EVENT_BUTTON_DOWN) {
            	if (Screen.ActualScreen == SCREEN_5 && CoolingRequest == 1){
            		if (selectedBox == 0 || selectedBox == 1) {
            			selectedBox += 2;
            		} else if (selectedBox == 2 || selectedBox == 3) {
            	        selectedBox = 4;
            		} else if (selectedBox == 4) {
            	        selectedBox = 0;
            	    } else if(Screen.ActualScreen == SCREEN_5){
            	    	Screen.ActualScreen = SCREEN_1;
            	    } else {
            	    	Screen.ActualScreen ++;
            	    }
            	}
            }

            if (pendingButtonEvent == EVENT_BUTTON_UP) {

            	if ((Screen.ActualScreen = SCREEN_5) && (CoolingRequest == 1)){
            		if (selectedBox == 4) {
            			selectedBox = 2;

            		} else if (selectedBox == 2 || selectedBox == 3) {
            			selectedBox -= 2;

            		} else if (selectedBox == 0 || selectedBox == 1) {
            			selectedBox = 4;

            		} else if (Screen.ActualScreen == SCREEN_1){
            			Screen.ActualScreen = SCREEN_5;

            		} else {
            			Screen.ActualScreen --;

            		}
            	}
            }

            if (pendingButtonEvent == EVENT_BUTTON_RIGHT) {
            	if (Screen.ActualScreen == SCREEN_5 && CoolingRequest == 1){
            	    selectedBox = (selectedBox + 1) % 5;  // Mueve a la derecha
            	}
            }

            if (pendingButtonEvent == EVENT_BUTTON_LEFT) {
            	if (Screen.ActualScreen == SCREEN_5 && CoolingRequest == 1){
            		if (selectedBox == 0) {
            		    selectedBox = 4;
            		} else {
            		    selectedBox = selectedBox - 1;
            		}
            	}
            }

            if (pendingButtonEvent == EVENT_BUTTON_OK) {
            	if (Screen.ActualScreen == SCREEN_5 && CoolingRequest == 0){
            		CoolingRequest = 1;
            		selectedBox = 0;
            	} else if (Screen.ActualScreen == SCREEN_5 && CoolingRequest == 1) {
            		switch (selectedBox) {
                    	case 0: Fans_R = (Fans_R + 1) % 3; break;
                    	case 1: Fans_L = (Fans_L + 1) % 3; break;
                    	case 2: Pump_R = (Pump_R + 1) % 2; break;
                    	case 3: Pump_L = (Pump_L + 1) % 2; break;
                    	case 4: CoolingRequest = 0; break;  			// Salir del modo edición
            		}
            	} else {
            		PrechargeRequest = 1;
            	}
            }
            break;

            /*
             * 	CAR STATE 2 - INVERTERS GETTING READY
             * */

        case DASH_2_PRECHARGE_STATUS:

            if (pendingButtonEvent == EVENT_BUTTON_DOWN) {

            }
            if (pendingButtonEvent == EVENT_BUTTON_UP) {

            }
            if (pendingButtonEvent == EVENT_BUTTON_RIGHT) {

            }
            if (pendingButtonEvent == EVENT_BUTTON_LEFT) {

            }
            if (pendingButtonEvent == EVENT_BUTTON_OK) {

            }
            break;

            /*
             * 	CAR STATE 3 - INVERTERS GETTING READY
             * */

        case DASH_3_PRECHARGE_FINISHED:

            if (pendingButtonEvent == EVENT_BUTTON_DOWN) {

            }
            if (pendingButtonEvent == EVENT_BUTTON_UP) {

            }
            if (pendingButtonEvent == EVENT_BUTTON_RIGHT) {

            }
            if (pendingButtonEvent == EVENT_BUTTON_LEFT) {

            }
            if (pendingButtonEvent == EVENT_BUTTON_OK) {

            }
            break;

            /*
             * 	CAR STATE 4 - INVERTERS GETTING READY
             * */

        case DASH_4_RACING_MENU:

            if (pendingButtonEvent == EVENT_BUTTON_DOWN) {
            	if (Screen.ActualScreen >= SCREEN_1 && Screen.ActualScreen <= SCREEN_5){
            		if (Screen.ActualScreen == SCREEN_5){
            			Screen.ActualScreen = SCREEN_1;
            		    RacingMode = 1;
            		 } else {
            		    Screen.ActualScreen ++;
            		    RacingMode ++;
            		 }
            	} else if (Screen.ActualScreen >= SCREEN_6 && Screen.ActualScreen <= SCREEN_10){
            		if (Screen.ActualScreen == SCREEN_10){
            		    Screen.ActualScreen = SCREEN_6;
            		    Driver = 1;
            		 } else {
            		    Screen.ActualScreen ++;
            		    Driver ++;
            		 }
            	}
            }

            if (pendingButtonEvent == EVENT_BUTTON_UP) {
            	if (Screen.ActualScreen >= SCREEN_1 && Screen.ActualScreen <= SCREEN_5){
            		if (Screen.ActualScreen == SCREEN_1){
            			Screen.ActualScreen = SCREEN_5;
            		    RacingMode = 5;
            		 } else {
            		    Screen.ActualScreen --;
            		    RacingMode --;
            		 }
            	} else if (Screen.ActualScreen >= SCREEN_6 && Screen.ActualScreen <= SCREEN_10){
            		if (Screen.ActualScreen == SCREEN_6){
            		    Screen.ActualScreen = SCREEN_10;
            		    Driver = 5;
            		 } else {
            		    Screen.ActualScreen --;
            		    Driver --;
            		 }
            	}
            }

            if (pendingButtonEvent == EVENT_BUTTON_RIGHT) {
            	if (Screen.ActualScreen >= SCREEN_1 && Screen.ActualScreen <= SCREEN_5){
            		Screen.ActualScreen = Driver + 5;
            	}
            }

            if (pendingButtonEvent == EVENT_BUTTON_LEFT) {
                 if (Screen.ActualScreen >= SCREEN_6 && Screen.ActualScreen <= SCREEN_10){
            		Screen.ActualScreen = RacingMode - 1;
            	}
            }

            if (pendingButtonEvent == EVENT_BUTTON_OK) {

            	if (Screen.ActualScreen == SCREEN_11){
            		EnableDrive_Order = 1;
            	}
            	if (Screen.ActualScreen >= SCREEN_6 && Screen.ActualScreen <= SCREEN_10){
            		Screen.ActualScreen = SCREEN_11;
            		RacingMode_Send = 1;
            	}
            }

            break;

            /*
             * 	CAR STATE 5 - INVERTERS GETTING READY
             * */

        case DASH_5_INVERTERS:

            if (pendingButtonEvent == EVENT_BUTTON_RIGHT) {

            }
            break;

            /*
             * 	CAR STATE 6 - RACING MODE
             * */

        case DASH_6_RACING_MODE:

        	// ACC, WORKSHOP,
            if (pendingButtonEvent == EVENT_ROTARY_1) {
            	TC_Level = currentRotaryState_1;
            }
            if (pendingButtonEvent == EVENT_ROTARY_2) {
            	TV_Level = currentRotaryState_2;
            }
            break;

            /*
             * 	CAR STATE 7 - ERRORES
             * */

        case DASH_7_ERROR:

            if (pendingButtonEvent == EVENT_BUTTON_RIGHT) {
            }
            break;

    }

    pendingButtonEvent = EVENT_NONE;

}
