#include "CAN_X_2025.h"

// Variables globals
CAN_TxHeaderTypeDef TxHeader = {0};
uint32_t TxMailbox = 0;
uint8_t TxData[8] = {0};
uint8_t error_count;

//Variables----------------------------------------------------------------------------------------------

uint8_t ETAS_Warning;
uint8_t el_AUTO_STATUS;
int8_t Inv_R_Icommand;
int8_t Inv_R_Iactual;
int8_t Inv_L_Icommand;
int8_t Inv_L_Iactual;
uint16_t SOE;
uint16_t ETAS_MSG_Counter;
uint16_t Highest_CellTemp;
uint16_t SOC_High;
uint16_t Lowest_CellVoltage;
uint8_t DeltaSOC_LastLap;
uint32_t Accu_Current;
uint8_t Reg_Level;
uint8_t BB_Dash;
uint8_t Disable_Regen;
uint8_t Sensorics_Mode;
uint8_t tel_DeltaSOC_LastLap;

uint16_t BrakePressure2;
uint16_t BrakePressure1;
uint32_t Accu_Current;
uint8_t Warning_Low_Voltage;
uint8_t Warning_Inv_Temp;
uint8_t Warning_Mot_Temp;
uint8_t Warning_Accu_Temp;
uint8_t tel_VoltageCell_Lowest;
uint16_t tel_AccuVoltage;
uint16_t tel_SOC_Low;
int8_t tel_IqR;
int8_t tel_IqL;
uint8_t tel_VEL;
uint16_t tel_TempCell_Highest;
uint16_t tel_TempMotor_R;
uint16_t tel_TempIGBT_R;
uint16_t tel_TempMotor_L;
uint16_t tel_TempIGBT_L;
uint8_t BMS_Alive;
uint8_t BMS_Balancing_Enable;
uint8_t BMS_Charge_Flag;
uint8_t BMS_OK;
uint8_t BMS_CAN_Disconnection;
uint8_t BMS_Voltage_Disconnection;
uint8_t BMS_NTC_Disconnection;
uint8_t BMS_Current_Disconnection;
uint8_t BMS_Undertemperature;
uint8_t BMS_Overtemperature;
uint8_t BMS_Overvoltage;
uint8_t BMS_Undervoltage;
uint8_t Precharge_State;
uint8_t AIR_Plus_State;
uint8_t AIR_Minus_State;
uint8_t Shutdown_PackageIntck;
uint32_t Charger_Accumulator_Current;
uint16_t Charger_Lowest_Cell_Temperature;
uint16_t Charger_Highest_Cell_Temperature;
uint16_t Charger_Average_Cell_Temperature;
uint32_t Charger_Acumulator_Voltage;
uint16_t Charger_Highest_Cell_Voltage;
uint16_t Charger_Lowest_Cell_Voltage;
uint8_t Charger_PrechargeOK;
uint8_t Charger_AIRs_State;
uint8_t Charger_AIRs_Request;
uint8_t Charger_Sync;
uint8_t VDC_RB_TqD;
uint8_t VDC_RB_TqBP;
uint8_t VDC_BP_SatUp;
uint8_t VDC_BP_SatDown;
uint8_t VDC_AP_SatRB;
uint8_t VDC_Steering_Deadzone;
uint8_t VDC_Min_Tyre_Slip;
uint8_t VDC_Max_Tyre_Slip;
uint8_t VDC_Max_TV_DiffTq;
uint8_t VDC_Max_Steering_Angle;
uint8_t VDC_AP_SatUp;
uint8_t VDC_AP_SatDown;
uint8_t RemainLaps;
uint8_t DeltaSOC_LastLap;
uint8_t AvgVEL_LastLap;
uint16_t TotalTime;
uint8_t LapCount;
uint16_t LapTime;
uint16_t Average_CellTemp;
uint32_t Accumulator_Voltage;
uint16_t Accumulator_Current;
uint8_t Shutdown_PackageIntck;
uint8_t Shutdown_IMD;
uint8_t Shutdown_BMS;
uint8_t AIRs_State;
uint16_t Highest_CellVoltage;
uint16_t Highest_CellTemp;
uint16_t Lowest_CellVoltage;
uint16_t Lowest_CellTemp;
uint16_t DASH_LV;
uint16_t DASH_5V;
uint16_t DASH_3V3;
uint8_t Dash_Alive;
uint8_t IMD_OK;
uint8_t BMS_OK;
uint16_t BMS_LV;
uint16_t BMS_5V;
uint16_t BMS_12V;
uint8_t BMS_Alive;
uint8_t Precharge_Percentage;
int16_t Power_Accu;
uint16_t SOC_High;
uint16_t SOC_Low;
uint16_t SOC_Avg;
uint8_t el_Track_Valid;
uint8_t el_Slip_Valid;
uint8_t el_Curvature_Valid;
int16_t el_SlipAngle;
uint16_t el_CurvatureRadius;
int16_t el_AngleTrack;
int16_t el_Vel_GPS_N;
int16_t el_Vel_GPS_E;
int16_t el_Vel_GPS_D;
uint8_t InvertersAction;
uint8_t Relay_Error;
uint8_t Regen_Enabled;
uint8_t TC_Warning;
uint8_t Sensorics_Mode;
uint8_t el_Vel_OK;
uint8_t Torque_OK;
uint8_t APPS_Implausibility;
uint8_t Disconnection_BrakePressure2;
uint8_t Disconnection_BrakePressure1;
uint8_t Critical_Signal_Disconnection;
uint8_t Critical_CAN_Disconnection;
uint8_t Disconnection_InvR;
uint8_t Disconnection_InvL;
uint8_t Disconnection_Susp_R_R;
uint8_t Disconnection_Susp_R_L;
uint8_t Disconnection_Susp_F_R;
uint8_t Disconnection_Susp_F_L;
uint8_t Disconnection_SteeringSensor;
uint8_t Disconnection_Rear;
uint8_t Disconnection_Pitot;
uint8_t Disconnection_Front;
uint8_t Disconnection_Ellipse;
uint8_t Disconnection_DashBoard;
uint8_t Disconnection_BrakePedal;
uint8_t Disconnection_BMS;
uint8_t Disconnection_APPS2;
uint8_t Disconnection_APPS1;
uint8_t Inv_Speed;
uint16_t el_VEL;
int8_t SteeringSensor_Value;
uint8_t BrakePedal_Value;
uint8_t APPS2_Value;
uint8_t APPS1_Value;
uint8_t Refri_ACCU;
uint8_t Pump_R;
uint8_t Pump_L;
uint8_t Button_2;
uint8_t Button_1;
uint8_t TV_Level;
uint8_t TC_Level;
uint8_t Driver;
uint8_t RacingMode;
uint8_t EnableDrive_Order;
uint8_t PrechargeRequest;
uint8_t Fans_R;
uint8_t Fans_L;
uint8_t Car_State;
uint8_t Car_OK;
uint8_t AIRs_Request;
uint8_t BrakeLight_Control;
int16_t el_Accel_Z;
int16_t el_Accel_Y;
int16_t el_Accel_X;
int16_t el_YAW;
int16_t el_Pitch;
int16_t el_Roll;
int16_t el_Vel_EKF_Z;
int16_t el_Vel_EKF_Y;
int16_t el_Vel_EKF_X;
int16_t el_Gyro_Z;
int16_t el_Gyro_Y;
int16_t el_Gyro_X;
uint8_t el_SolutionMode;
uint8_t el_Vel_GPS_Valid;
uint8_t el_Vel_EKF_Valid;
uint16_t Inv_R_TempMotor;
int16_t current_R;
uint16_t Inv_R_TempIGBT;
int8_t RR_Vel_ms_Wheel;
uint16_t Inv_L_TempMotor;
int16_t current_L;
uint16_t Inv_L_TempIGBT;
int8_t RL_Vel_ms_Wheel;
uint16_t SUSP_R_R;
uint16_t SUSP_R_L;
uint16_t SUSP_F_R;
uint16_t SUSP_F_L;
int16_t Throttle_Torque;
int16_t Torque_R;
int16_t Torque_L;
uint16_t Susp_R_R_Bits;
uint16_t Susp_R_L_Bits;
uint16_t FECU_LV;
uint16_t FECU_5V;
uint16_t FECU_3V3;
uint8_t Shutdown_Setas;
uint8_t Shutdown_SC_BOTS;
uint8_t Shutdown_BSPD_Inertia;
uint8_t Front_Alive;
uint16_t RECU_LV;
uint16_t RECU_5V;
uint16_t RECU_3V3;
uint8_t Shutdown_TSMS_TSMP;
uint8_t Shutdown_RightTS;
uint8_t Shutdown_LeftTS;
uint8_t Shutdown_HVD;
uint8_t Shutdown_HVBox;
uint8_t Rear_Alive;
uint16_t BrakePressure2_Bits;
uint16_t BrakePressure1_Bits;
uint16_t Pitot_Bits;
uint16_t Susp_F_R_Bits;
uint16_t Susp_F_L_Bits;
uint16_t SteeringSensor_Bits;
uint16_t BrakePedal_Bits;
uint16_t APPS2_Bits;
uint16_t APPS1_Bits;
uint8_t ETAS_Sync_X;

//Tx functions-------------------------------------------------------------------------------------------

void message_cantx_VECTOR__INDEPENDENT_SIG_MSG(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 0;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = VECTOR__INDEPENDENT_SIG_MSG_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[0] = (TxData[0] & ~1) | (((ETAS_Warning >> 1) << 0) & 1);
   TxData[1] = (TxData[1] & ~128) | ((ETAS_Warning << 7) & 128); 
   TxData[0] = (TxData[0] & ~1) | (((el_AUTO_STATUS >> 7) << 0) & 1);
   TxData[1] = (TxData[1] & ~254) | ((el_AUTO_STATUS << 1) & 254); 
   TxData[0] = (TxData[0] & ~1) | (((Inv_R_Icommand >> 7) << 0) & 1);
   TxData[1] = (TxData[1] & ~254) | ((Inv_R_Icommand << 1) & 254); 
   TxData[0] = (TxData[0] & ~1) | (((Inv_R_Iactual >> 7) << 0) & 1);
   TxData[1] = (TxData[1] & ~254) | ((Inv_R_Iactual << 1) & 254); 
   TxData[0] = (TxData[0] & ~1) | (((Inv_L_Icommand >> 7) << 0) & 1);
   TxData[1] = (TxData[1] & ~254) | ((Inv_L_Icommand << 1) & 254); 
   TxData[0] = (TxData[0] & ~1) | (((Inv_L_Iactual >> 7) << 0) & 1);
   TxData[1] = (TxData[1] & ~254) | ((Inv_L_Iactual << 1) & 254); 
   TxData[0] = (TxData[0] & ~1) | (((SOE >> 9) << 0) & 1);
   TxData[1] = SOE >> 1;
   TxData[2] = (TxData[2] & ~128) | ((SOE << 7) & 128); 
   TxData[0] = (TxData[0] & ~1) | (((ETAS_MSG_Counter >> 15) << 0) & 1);
   TxData[1] = ETAS_MSG_Counter >> 7;
   TxData[2] = (TxData[2] & ~254) | ((ETAS_MSG_Counter << 1) & 254); 
   TxData[0] = (TxData[0] & ~1) | (((Highest_CellTemp >> 9) << 0) & 1);
   TxData[1] = Highest_CellTemp >> 1;
   TxData[2] = (TxData[2] & ~128) | ((Highest_CellTemp << 7) & 128); 
   TxData[0] = (TxData[0] & ~1) | (((SOC_High >> 9) << 0) & 1);
   TxData[1] = SOC_High >> 1;
   TxData[2] = (TxData[2] & ~128) | ((SOC_High << 7) & 128); 
   TxData[0] = (TxData[0] & ~1) | (((Lowest_CellVoltage >> 15) << 0) & 1);
   TxData[1] = Lowest_CellVoltage >> 7;
   TxData[2] = (TxData[2] & ~254) | ((Lowest_CellVoltage << 1) & 254); 
   TxData[0] = (TxData[0] & ~1) | (((DeltaSOC_LastLap >> 6) << 0) & 1);
   TxData[1] = (TxData[1] & ~252) | ((DeltaSOC_LastLap << 2) & 252); 
   TxData[0] = (TxData[0] & ~1) | (((Accu_Current >> 31) << 0) & 1);
   TxData[1] = Accu_Current >> 23;
   TxData[2] = Accu_Current >> 15;
   TxData[3] = Accu_Current >> 7;
   TxData[4] = (TxData[4] & ~254) | ((Accu_Current << 1) & 254); 
   TxData[0] = (TxData[0] & ~1) | (((Reg_Level >> 1) << 0) & 1);
   TxData[1] = (TxData[1] & ~128) | ((Reg_Level << 7) & 128); 
   TxData[0] = (TxData[0] & ~1) | (((BB_Dash >> 5) << 0) & 1);
   TxData[1] = (TxData[1] & ~248) | ((BB_Dash << 3) & 248); 
   TxData[0] = (TxData[0] & ~1) | (((Disable_Regen >> 0) << 0) & 1);
   TxData[0] = (TxData[0] & ~1) | (((Sensorics_Mode >> 1) << 0) & 1);
   TxData[1] = (TxData[1] & ~128) | ((Sensorics_Mode << 7) & 128); 
   TxData[0] = (TxData[0] & ~1) | (((tel_DeltaSOC_LastLap >> 6) << 0) & 1);
   TxData[1] = (TxData[1] & ~252) | ((tel_DeltaSOC_LastLap << 2) & 252); 
   TxData[0] = (TxData[0] & ~1) | (((AvgVEL_LastLap >> 9) << 0) & 1);
   TxData[1] = AvgVEL_LastLap >> 1;
   TxData[2] = (TxData[2] & ~128) | ((AvgVEL_LastLap << 7) & 128); 
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_PROC_ETAS_VDC_Pressure(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 3;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = PROC_ETAS_VDC_Pressure_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[1] = (TxData[1] & ~127) | (((BrakePressure2 >> 2) << 0) & 127);
   TxData[2] = (TxData[2] & ~192) | ((BrakePressure2 << 6) & 192); 
   TxData[0] = (TxData[0] & ~255) | (((BrakePressure1 >> 1) << 0) & 255);
   TxData[1] = (TxData[1] & ~128) | ((BrakePressure1 << 7) & 128); 
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_CAB300(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 4;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = CAB300_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[0] = (TxData[0] & ~255) | (((Accu_Current >> 24) << 0) & 255);
   TxData[1] = Accu_Current >> 16;
   TxData[2] = Accu_Current >> 8;
   TxData[3] = Accu_Current >> 0;
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_PROC_ETAS_Warning(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 1;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = PROC_ETAS_Warning_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[0] = (TxData[0] & ~64) | (((Warning_Low_Voltage >> 0) << 6) & 64);
   TxData[0] = (TxData[0] & ~32) | (((Warning_Inv_Temp >> 0) << 5) & 32);
   TxData[0] = (TxData[0] & ~16) | (((Warning_Mot_Temp >> 0) << 4) & 16);
   TxData[0] = (TxData[0] & ~128) | (((Warning_Accu_Temp >> 0) << 7) & 128);
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_TEL_ETAS_Accu(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 4;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = TEL_ETAS_Accu_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[2] = (TxData[2] & ~7) | (((tel_VoltageCell_Lowest >> 5) << 0) & 7);
   TxData[3] = (TxData[3] & ~248) | ((tel_VoltageCell_Lowest << 3) & 248); 
   TxData[1] = (TxData[1] & ~63) | (((tel_AccuVoltage >> 5) << 0) & 63);
   TxData[2] = (TxData[2] & ~248) | ((tel_AccuVoltage << 3) & 248); 
   TxData[0] = (TxData[0] & ~255) | (((tel_SOC_Low >> 2) << 0) & 255);
   TxData[1] = (TxData[1] & ~192) | ((tel_SOC_Low << 6) & 192); 
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_TEL_ETAS_Info(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 3;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = TEL_ETAS_Info_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[1] = (TxData[1] & ~255) | (((tel_IqR >> 0) << 0) & 255);
   TxData[2] = (TxData[2] & ~255) | (((tel_IqL >> 0) << 0) & 255);
   TxData[0] = (TxData[0] & ~255) | (((tel_VEL >> 0) << 0) & 255);
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_TEL_ETAS_Temps(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 7;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = TEL_ETAS_Temps_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[5] = (TxData[5] & ~63) | (((tel_TempCell_Highest >> 4) << 0) & 63);
   TxData[6] = (TxData[6] & ~240) | ((tel_TempCell_Highest << 4) & 240); 
   TxData[3] = (TxData[3] & ~1) | (((tel_TempMotor_R >> 10) << 0) & 1);
   TxData[4] = tel_TempMotor_R >> 2;
   TxData[5] = (TxData[5] & ~192) | ((tel_TempMotor_R << 6) & 192); 
   TxData[2] = (TxData[2] & ~7) | (((tel_TempIGBT_R >> 7) << 0) & 7);
   TxData[3] = (TxData[3] & ~254) | ((tel_TempIGBT_R << 1) & 254); 
   TxData[1] = (TxData[1] & ~63) | (((tel_TempMotor_L >> 5) << 0) & 63);
   TxData[2] = (TxData[2] & ~248) | ((tel_TempMotor_L << 3) & 248); 
   TxData[0] = (TxData[0] & ~255) | (((tel_TempIGBT_L >> 2) << 0) & 255);
   TxData[1] = (TxData[1] & ~192) | ((tel_TempIGBT_L << 6) & 192); 
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_NM_BMS_Charger_Keep_Alive(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 1;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = NM_BMS_Charger_Keep_Alive_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[0] = (TxData[0] & ~255) | (((BMS_Alive >> 0) << 0) & 255);
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_STAT_BMS_SFR(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 3;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = STAT_BMS_SFR_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[2] = (TxData[2] & ~64) | (((BMS_Balancing_Enable >> 0) << 6) & 64);
   TxData[2] = (TxData[2] & ~128) | (((BMS_Charge_Flag >> 0) << 7) & 128);
   TxData[1] = (TxData[1] & ~8) | (((BMS_OK >> 0) << 3) & 8);
   TxData[1] = (TxData[1] & ~16) | (((BMS_CAN_Disconnection >> 0) << 4) & 16);
   TxData[1] = (TxData[1] & ~32) | (((BMS_Voltage_Disconnection >> 0) << 5) & 32);
   TxData[1] = (TxData[1] & ~64) | (((BMS_NTC_Disconnection >> 0) << 6) & 64);
   TxData[1] = (TxData[1] & ~128) | (((BMS_Current_Disconnection >> 0) << 7) & 128);
   TxData[0] = (TxData[0] & ~1) | (((BMS_Undertemperature >> 0) << 0) & 1);
   TxData[0] = (TxData[0] & ~2) | (((BMS_Overtemperature >> 0) << 1) & 2);
   TxData[0] = (TxData[0] & ~4) | (((BMS_Overvoltage >> 0) << 2) & 4);
   TxData[0] = (TxData[0] & ~8) | (((BMS_Undervoltage >> 0) << 3) & 8);
   TxData[0] = (TxData[0] & ~16) | (((Precharge_State >> 0) << 4) & 16);
   TxData[0] = (TxData[0] & ~32) | (((AIR_Plus_State >> 0) << 5) & 32);
   TxData[0] = (TxData[0] & ~64) | (((AIR_Minus_State >> 0) << 6) & 64);
   TxData[0] = (TxData[0] & ~128) | (((Shutdown_PackageIntck >> 0) << 7) & 128);
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_CHARGER_Battery_Current(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 3;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = CHARGER_Battery_Current_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[0] = (TxData[0] & ~255) | (((Charger_Accumulator_Current >> 16) << 0) & 255);
   TxData[1] = Charger_Accumulator_Current >> 8;
   TxData[2] = Charger_Accumulator_Current >> 0;
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_STAT_CHARGER_Battery_Temp(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 6;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = STAT_CHARGER_Battery_Temp_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[0] = (TxData[0] & ~255) | (((Charger_Lowest_Cell_Temperature >> 8) << 0) & 255);
   TxData[1] = Charger_Lowest_Cell_Temperature >> 0;
   TxData[2] = (TxData[2] & ~255) | (((Charger_Highest_Cell_Temperature >> 8) << 0) & 255);
   TxData[3] = Charger_Highest_Cell_Temperature >> 0;
   TxData[4] = (TxData[4] & ~255) | (((Charger_Average_Cell_Temperature >> 8) << 0) & 255);
   TxData[5] = Charger_Average_Cell_Temperature >> 0;
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_STAT_CHARGER_Battery_Voltages(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 7;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = STAT_CHARGER_Battery_Voltages_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[4] = (TxData[4] & ~255) | (((Charger_Acumulator_Voltage >> 16) << 0) & 255);
   TxData[5] = Charger_Acumulator_Voltage >> 8;
   TxData[6] = Charger_Acumulator_Voltage >> 0;
   TxData[2] = (TxData[2] & ~255) | (((Charger_Highest_Cell_Voltage >> 8) << 0) & 255);
   TxData[3] = Charger_Highest_Cell_Voltage >> 0;
   TxData[0] = (TxData[0] & ~255) | (((Charger_Lowest_Cell_Voltage >> 8) << 0) & 255);
   TxData[1] = Charger_Lowest_Cell_Voltage >> 0;
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_STAT_CHARGER_PrechargeOK(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 1;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = STAT_CHARGER_PrechargeOK_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[0] = (TxData[0] & ~255) | (((Charger_PrechargeOK >> 0) << 0) & 255);
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_STAT_BMS_AIRs_State(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 1;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = STAT_BMS_AIRs_State_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[0] = (TxData[0] & ~15) | (((Charger_AIRs_State >> 0) << 0) & 15);
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_CTRL_CHARGER_AIRs_Request(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 1;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = CTRL_CHARGER_AIRs_Request_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[0] = (TxData[0] & ~15) | (((Charger_AIRs_Request >> 0) << 0) & 15);
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_STAT_CHARGER_Sync(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 1;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = STAT_CHARGER_Sync_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[0] = (TxData[0] & ~255) | (((Charger_Sync >> 0) << 0) & 255);
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_PROC_ETAS_VDC_Params(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 8;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = PROC_ETAS_VDC_Params_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[7] = (TxData[7] & ~240) | (((VDC_RB_TqD >> 0) << 4) & 240);
   TxData[7] = (TxData[7] & ~15) | (((VDC_RB_TqBP >> 0) << 0) & 15);
   TxData[6] = (TxData[6] & ~63) | (((VDC_BP_SatUp >> 0) << 0) & 63);
   TxData[5] = (TxData[5] & ~15) | (((VDC_BP_SatDown >> 2) << 0) & 15);
   TxData[6] = (TxData[6] & ~192) | ((VDC_BP_SatDown << 6) & 192); 
   TxData[4] = (TxData[4] & ~3) | (((VDC_AP_SatRB >> 4) << 0) & 3);
   TxData[5] = (TxData[5] & ~240) | ((VDC_AP_SatRB << 4) & 240); 
   TxData[3] = (TxData[3] & ~248) | (((VDC_Steering_Deadzone >> 0) << 3) & 248);
   TxData[0] = (TxData[0] & ~3) | (((VDC_Min_Tyre_Slip >> 4) << 0) & 3);
   TxData[1] = (TxData[1] & ~240) | ((VDC_Min_Tyre_Slip << 4) & 240); 
   TxData[0] = (TxData[0] & ~252) | (((VDC_Max_Tyre_Slip >> 0) << 2) & 252);
   TxData[4] = (TxData[4] & ~124) | (((VDC_Max_TV_DiffTq >> 0) << 2) & 124);
   TxData[3] = (TxData[3] & ~7) | (((VDC_Max_Steering_Angle >> 1) << 0) & 7);
   TxData[4] = (TxData[4] & ~128) | ((VDC_Max_Steering_Angle << 7) & 128); 
   TxData[2] = (TxData[2] & ~63) | (((VDC_AP_SatUp >> 0) << 0) & 63);
   TxData[1] = (TxData[1] & ~15) | (((VDC_AP_SatDown >> 2) << 0) & 15);
   TxData[2] = (TxData[2] & ~192) | ((VDC_AP_SatDown << 6) & 192); 
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_PROC_ETAS_VDC_LapTiming(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 6;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = PROC_ETAS_VDC_LapTiming_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[5] = (TxData[5] & ~62) | (((RemainLaps >> 0) << 1) & 62);
   TxData[4] = (TxData[4] & ~31) | (((DeltaSOC_LastLap >> 2) << 0) & 31);
   TxData[5] = (TxData[5] & ~192) | ((DeltaSOC_LastLap << 6) & 192); 
   TxData[3] = (TxData[3] & ~31) | (((AvgVEL_LastLap >> 3) << 0) & 31);
   TxData[4] = (TxData[4] & ~224) | ((AvgVEL_LastLap << 5) & 224); 
   TxData[2] = (TxData[2] & ~255) | (((TotalTime >> 3) << 0) & 255);
   TxData[3] = (TxData[3] & ~224) | ((TotalTime << 5) & 224); 
   TxData[1] = (TxData[1] & ~31) | (((LapCount >> 0) << 0) & 31);
   TxData[0] = (TxData[0] & ~255) | (((LapTime >> 3) << 0) & 255);
   TxData[1] = (TxData[1] & ~224) | ((LapTime << 5) & 224); 
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_CTRL_BMS_Accu_Data(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 6;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = CTRL_BMS_Accu_Data_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[4] = (TxData[4] & ~3) | (((Average_CellTemp >> 8) << 0) & 3);
   TxData[5] = Average_CellTemp >> 0;
   TxData[0] = (TxData[0] & ~7) | (((Accumulator_Voltage >> 18) << 0) & 7);
   TxData[1] = Accumulator_Voltage >> 10;
   TxData[2] = Accumulator_Voltage >> 2;
   TxData[3] = (TxData[3] & ~192) | ((Accumulator_Voltage << 6) & 192); 
   TxData[3] = (TxData[3] & ~63) | (((Accumulator_Current >> 6) << 0) & 63);
   TxData[4] = (TxData[4] & ~252) | ((Accumulator_Current << 2) & 252); 
   TxData[0] = (TxData[0] & ~32) | (((Shutdown_PackageIntck >> 0) << 5) & 32);
   TxData[0] = (TxData[0] & ~8) | (((Shutdown_IMD >> 0) << 3) & 8);
   TxData[0] = (TxData[0] & ~16) | (((Shutdown_BMS >> 0) << 4) & 16);
   TxData[0] = (TxData[0] & ~192) | (((AIRs_State >> 0) << 6) & 192);
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_CTRL_BMS_Cell_Extremes(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 7;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = CTRL_BMS_Cell_Extremes_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[2] = (TxData[2] & ~255) | (((Highest_CellVoltage >> 8) << 0) & 255);
   TxData[3] = Highest_CellVoltage >> 0;
   TxData[5] = (TxData[5] & ~63) | (((Highest_CellTemp >> 4) << 0) & 63);
   TxData[6] = (TxData[6] & ~240) | ((Highest_CellTemp << 4) & 240); 
   TxData[0] = (TxData[0] & ~255) | (((Lowest_CellVoltage >> 8) << 0) & 255);
   TxData[1] = Lowest_CellVoltage >> 0;
   TxData[4] = (TxData[4] & ~255) | (((Lowest_CellTemp >> 2) << 0) & 255);
   TxData[5] = (TxData[5] & ~192) | ((Lowest_CellTemp << 6) & 192); 
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_STAT_DASH_Keep_Alive(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 6;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = STAT_DASH_Keep_Alive_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[4] = (TxData[4] & ~255) | (((DASH_LV >> 4) << 0) & 255);
   TxData[5] = (TxData[5] & ~240) | ((DASH_LV << 4) & 240); 
   TxData[2] = (TxData[2] & ~15) | (((DASH_5V >> 8) << 0) & 15);
   TxData[3] = DASH_5V >> 0;
   TxData[1] = (TxData[1] & ~255) | (((DASH_3V3 >> 4) << 0) & 255);
   TxData[2] = (TxData[2] & ~240) | ((DASH_3V3 << 4) & 240); 
   TxData[0] = (TxData[0] & ~255) | (((Dash_Alive >> 0) << 0) & 255);
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_STAT_BMS_Keep_Alive(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 6;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = STAT_BMS_Keep_Alive_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[5] = (TxData[5] & ~4) | (((IMD_OK >> 0) << 2) & 4);
   TxData[5] = (TxData[5] & ~8) | (((BMS_OK >> 0) << 3) & 8);
   TxData[4] = (TxData[4] & ~255) | (((BMS_LV >> 4) << 0) & 255);
   TxData[5] = (TxData[5] & ~240) | ((BMS_LV << 4) & 240); 
   TxData[2] = (TxData[2] & ~15) | (((BMS_5V >> 8) << 0) & 15);
   TxData[3] = BMS_5V >> 0;
   TxData[1] = (TxData[1] & ~255) | (((BMS_12V >> 4) << 0) & 255);
   TxData[2] = (TxData[2] & ~240) | ((BMS_12V << 4) & 240); 
   TxData[0] = (TxData[0] & ~255) | (((BMS_Alive >> 0) << 0) & 255);
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_PROC_ETAS_TS_Data(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 7;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = PROC_ETAS_TS_Data_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[5] = (TxData[5] & ~15) | (((Precharge_Percentage >> 3) << 0) & 15);
   TxData[6] = (TxData[6] & ~224) | ((Precharge_Percentage << 5) & 224); 
   TxData[3] = (TxData[3] & ~3) | (((Power_Accu >> 12) << 0) & 3);
   TxData[4] = Power_Accu >> 4;
   TxData[5] = (TxData[5] & ~240) | ((Power_Accu << 4) & 240); 
   TxData[2] = (TxData[2] & ~15) | (((SOC_High >> 6) << 0) & 15);
   TxData[3] = (TxData[3] & ~252) | ((SOC_High << 2) & 252); 
   TxData[1] = (TxData[1] & ~63) | (((SOC_Low >> 4) << 0) & 63);
   TxData[2] = (TxData[2] & ~240) | ((SOC_Low << 4) & 240); 
   TxData[0] = (TxData[0] & ~255) | (((SOC_Avg >> 2) << 0) & 255);
   TxData[1] = (TxData[1] & ~192) | ((SOC_Avg << 6) & 192); 
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_PROC_ELLIPSE_Auto(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 7;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = PROC_ELLIPSE_Auto_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[6] = (TxData[6] & ~1) | (((el_Track_Valid >> 0) << 0) & 1);
   TxData[6] = (TxData[6] & ~2) | (((el_Slip_Valid >> 0) << 1) & 2);
   TxData[6] = (TxData[6] & ~4) | (((el_Curvature_Valid >> 0) << 2) & 4);
   TxData[2] = (TxData[2] & ~1) | (((el_SlipAngle >> 15) << 0) & 1);
   TxData[3] = el_SlipAngle >> 7;
   TxData[4] = (TxData[4] & ~254) | ((el_SlipAngle << 1) & 254); 
   TxData[4] = (TxData[4] & ~1) | (((el_CurvatureRadius >> 15) << 0) & 1);
   TxData[5] = el_CurvatureRadius >> 7;
   TxData[6] = (TxData[6] & ~254) | ((el_CurvatureRadius << 1) & 254); 
   TxData[0] = (TxData[0] & ~1) | (((el_AngleTrack >> 15) << 0) & 1);
   TxData[1] = el_AngleTrack >> 7;
   TxData[2] = (TxData[2] & ~254) | ((el_AngleTrack << 1) & 254); 
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_CTRL_ELLIPSE_GPS_Vel(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 6;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = CTRL_ELLIPSE_GPS_Vel_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[0] = (TxData[0] & ~1) | (((el_Vel_GPS_N >> 15) << 0) & 1);
   TxData[1] = el_Vel_GPS_N >> 7;
   TxData[2] = (TxData[2] & ~254) | ((el_Vel_GPS_N << 1) & 254); 
   TxData[2] = (TxData[2] & ~1) | (((el_Vel_GPS_E >> 15) << 0) & 1);
   TxData[3] = el_Vel_GPS_E >> 7;
   TxData[4] = (TxData[4] & ~254) | ((el_Vel_GPS_E << 1) & 254); 
   TxData[4] = (TxData[4] & ~1) | (((el_Vel_GPS_D >> 15) << 0) & 1);
   TxData[5] = el_Vel_GPS_D >> 7;
   TxData[6] = (TxData[6] & ~254) | ((el_Vel_GPS_D << 1) & 254); 
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_STAT_ETAS_Diagnostics(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 5;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = STAT_ETAS_Diagnostics_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[3] = (TxData[3] & ~14) | (((InvertersAction >> 0) << 1) & 14);
   TxData[3] = (TxData[3] & ~64) | (((Relay_Error >> 0) << 6) & 64);
   TxData[3] = (TxData[3] & ~128) | (((Regen_Enabled >> 0) << 7) & 128);
   TxData[2] = (TxData[2] & ~1) | (((TC_Warning >> 0) << 0) & 1);
   TxData[3] = (TxData[3] & ~48) | (((Sensorics_Mode >> 0) << 4) & 48);
   TxData[2] = (TxData[2] & ~2) | (((el_Vel_OK >> 0) << 1) & 2);
   TxData[2] = (TxData[2] & ~4) | (((Torque_OK >> 0) << 2) & 4);
   TxData[2] = (TxData[2] & ~8) | (((APPS_Implausibility >> 0) << 3) & 8);
   TxData[2] = (TxData[2] & ~16) | (((Disconnection_BrakePressure2 >> 0) << 4) & 16);
   TxData[2] = (TxData[2] & ~32) | (((Disconnection_BrakePressure1 >> 0) << 5) & 32);
   TxData[0] = (TxData[0] & ~128) | (((Critical_Signal_Disconnection >> 0) << 7) & 128);
   TxData[0] = (TxData[0] & ~64) | (((Critical_CAN_Disconnection >> 0) << 6) & 64);
   TxData[2] = (TxData[2] & ~64) | (((Disconnection_InvR >> 0) << 6) & 64);
   TxData[2] = (TxData[2] & ~128) | (((Disconnection_InvL >> 0) << 7) & 128);
   TxData[1] = (TxData[1] & ~16) | (((Disconnection_Susp_R_R >> 0) << 4) & 16);
   TxData[1] = (TxData[1] & ~8) | (((Disconnection_Susp_R_L >> 0) << 3) & 8);
   TxData[1] = (TxData[1] & ~4) | (((Disconnection_Susp_F_R >> 0) << 2) & 4);
   TxData[1] = (TxData[1] & ~2) | (((Disconnection_Susp_F_L >> 0) << 1) & 2);
   TxData[1] = (TxData[1] & ~32) | (((Disconnection_SteeringSensor >> 0) << 5) & 32);
   TxData[0] = (TxData[0] & ~2) | (((Disconnection_Rear >> 0) << 1) & 2);
   TxData[1] = (TxData[1] & ~1) | (((Disconnection_Pitot >> 0) << 0) & 1);
   TxData[0] = (TxData[0] & ~8) | (((Disconnection_Front >> 0) << 3) & 8);
   TxData[0] = (TxData[0] & ~4) | (((Disconnection_Ellipse >> 0) << 2) & 4);
   TxData[0] = (TxData[0] & ~16) | (((Disconnection_DashBoard >> 0) << 4) & 16);
   TxData[1] = (TxData[1] & ~64) | (((Disconnection_BrakePedal >> 0) << 6) & 64);
   TxData[0] = (TxData[0] & ~32) | (((Disconnection_BMS >> 0) << 5) & 32);
   TxData[1] = (TxData[1] & ~128) | (((Disconnection_APPS2 >> 0) << 7) & 128);
   TxData[0] = (TxData[0] & ~1) | (((Disconnection_APPS1 >> 0) << 0) & 1);
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_PROC_ETAS_VDC_Values(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 6;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = PROC_ETAS_VDC_Values_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[5] = (TxData[5] & ~127) | (((Inv_Speed >> 0) << 0) & 127);
   TxData[3] = (TxData[3] & ~7) | (((el_VEL >> 9) << 0) & 7);
   TxData[4] = el_VEL >> 1;
   TxData[5] = (TxData[5] & ~128) | ((el_VEL << 7) & 128); 
   TxData[2] = (TxData[2] & ~7) | (((SteeringSensor_Value >> 5) << 0) & 7);
   TxData[3] = (TxData[3] & ~248) | ((SteeringSensor_Value << 3) & 248); 
   TxData[1] = (TxData[1] & ~3) | (((BrakePedal_Value >> 5) << 0) & 3);
   TxData[2] = (TxData[2] & ~248) | ((BrakePedal_Value << 3) & 248); 
   TxData[0] = (TxData[0] & ~1) | (((APPS2_Value >> 6) << 0) & 1);
   TxData[1] = (TxData[1] & ~252) | ((APPS2_Value << 2) & 252); 
   TxData[0] = (TxData[0] & ~254) | (((APPS1_Value >> 0) << 1) & 254);
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_CTRL_DASH_Driver_Inputs(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 3;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = CTRL_DASH_Driver_Inputs_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[2] = (TxData[2] & ~3) | (((Refri_ACCU >> 0) << 0) & 3);
   TxData[2] = (TxData[2] & ~16) | (((Pump_R >> 0) << 4) & 16);
   TxData[2] = (TxData[2] & ~32) | (((Pump_L >> 0) << 5) & 32);
   TxData[2] = (TxData[2] & ~4) | (((Button_2 >> 0) << 2) & 4);
   TxData[2] = (TxData[2] & ~8) | (((Button_1 >> 0) << 3) & 8);
   TxData[1] = (TxData[1] & ~28) | (((TV_Level >> 0) << 2) & 28);
   TxData[1] = (TxData[1] & ~224) | (((TC_Level >> 0) << 5) & 224);
   TxData[0] = (TxData[0] & ~7) | (((Driver >> 0) << 0) & 7);
   TxData[0] = (TxData[0] & ~56) | (((RacingMode >> 0) << 3) & 56);
   TxData[0] = (TxData[0] & ~64) | (((EnableDrive_Order >> 0) << 6) & 64);
   TxData[0] = (TxData[0] & ~128) | (((PrechargeRequest >> 0) << 7) & 128);
   TxData[2] = (TxData[2] & ~192) | (((Fans_R >> 0) << 6) & 192);
   TxData[1] = (TxData[1] & ~3) | (((Fans_L >> 0) << 0) & 3);
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_CTRL_ETAS_System(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 1;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = CTRL_ETAS_System_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[0] = (TxData[0] & ~56) | (((Car_State >> 0) << 3) & 56);
   TxData[0] = (TxData[0] & ~2) | (((Car_OK >> 0) << 1) & 2);
   TxData[0] = (TxData[0] & ~192) | (((AIRs_Request >> 0) << 6) & 192);
   TxData[0] = (TxData[0] & ~4) | (((BrakeLight_Control >> 0) << 2) & 4);
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_PROC_ELLIPSE_EKF_Pos(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 8;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = PROC_ELLIPSE_EKF_Pos_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_PROC_ELLIPSE_IMU_Accel(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 6;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = PROC_ELLIPSE_IMU_Accel_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[4] = (TxData[4] & ~1) | (((el_Accel_Z >> 15) << 0) & 1);
   TxData[5] = el_Accel_Z >> 7;
   TxData[6] = (TxData[6] & ~254) | ((el_Accel_Z << 1) & 254); 
   TxData[2] = (TxData[2] & ~1) | (((el_Accel_Y >> 15) << 0) & 1);
   TxData[3] = el_Accel_Y >> 7;
   TxData[4] = (TxData[4] & ~254) | ((el_Accel_Y << 1) & 254); 
   TxData[0] = (TxData[0] & ~1) | (((el_Accel_X >> 15) << 0) & 1);
   TxData[1] = el_Accel_X >> 7;
   TxData[2] = (TxData[2] & ~254) | ((el_Accel_X << 1) & 254); 
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_PROC_ELLIPSE_EKF_Euler(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 6;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = PROC_ELLIPSE_EKF_Euler_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[4] = (TxData[4] & ~1) | (((el_YAW >> 15) << 0) & 1);
   TxData[5] = el_YAW >> 7;
   TxData[6] = (TxData[6] & ~254) | ((el_YAW << 1) & 254); 
   TxData[2] = (TxData[2] & ~1) | (((el_Pitch >> 15) << 0) & 1);
   TxData[3] = el_Pitch >> 7;
   TxData[4] = (TxData[4] & ~254) | ((el_Pitch << 1) & 254); 
   TxData[0] = (TxData[0] & ~1) | (((el_Roll >> 15) << 0) & 1);
   TxData[1] = el_Roll >> 7;
   TxData[2] = (TxData[2] & ~254) | ((el_Roll << 1) & 254); 
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_CTRL_ELLIPSE_EKF_Vel(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 6;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = CTRL_ELLIPSE_EKF_Vel_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[4] = (TxData[4] & ~1) | (((el_Vel_EKF_Z >> 15) << 0) & 1);
   TxData[5] = el_Vel_EKF_Z >> 7;
   TxData[6] = (TxData[6] & ~254) | ((el_Vel_EKF_Z << 1) & 254); 
   TxData[2] = (TxData[2] & ~1) | (((el_Vel_EKF_Y >> 15) << 0) & 1);
   TxData[3] = el_Vel_EKF_Y >> 7;
   TxData[4] = (TxData[4] & ~254) | ((el_Vel_EKF_Y << 1) & 254); 
   TxData[0] = (TxData[0] & ~1) | (((el_Vel_EKF_X >> 15) << 0) & 1);
   TxData[1] = el_Vel_EKF_X >> 7;
   TxData[2] = (TxData[2] & ~254) | ((el_Vel_EKF_X << 1) & 254); 
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_STAT_ELLIPSE_Status(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 4;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = STAT_ELLIPSE_Status_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_PROC_ELLIPSE_IMU_Gyro(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 6;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = PROC_ELLIPSE_IMU_Gyro_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[4] = (TxData[4] & ~1) | (((el_Gyro_Z >> 15) << 0) & 1);
   TxData[5] = el_Gyro_Z >> 7;
   TxData[6] = (TxData[6] & ~254) | ((el_Gyro_Z << 1) & 254); 
   TxData[2] = (TxData[2] & ~1) | (((el_Gyro_Y >> 15) << 0) & 1);
   TxData[3] = el_Gyro_Y >> 7;
   TxData[4] = (TxData[4] & ~254) | ((el_Gyro_Y << 1) & 254); 
   TxData[0] = (TxData[0] & ~1) | (((el_Gyro_X >> 15) << 0) & 1);
   TxData[1] = el_Gyro_X >> 7;
   TxData[2] = (TxData[2] & ~254) | ((el_Gyro_X << 1) & 254); 
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_CTRL_ELLIPSE_Vel_Valid(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 6;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = CTRL_ELLIPSE_Vel_Valid_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[0] = (TxData[0] & ~1) | (((el_SolutionMode >> 3) << 0) & 1);
   TxData[1] = (TxData[1] & ~224) | ((el_SolutionMode << 5) & 224); 
   TxData[1] = (TxData[1] & ~4) | (((el_Vel_GPS_Valid >> 0) << 2) & 4);
   TxData[0] = (TxData[0] & ~64) | (((el_Vel_EKF_Valid >> 0) << 6) & 64);
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_PROC_ETAS_Inverter_R_Data(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 5;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = PROC_ETAS_Inverter_R_Data_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[1] = (TxData[1] & ~63) | (((Inv_R_TempMotor >> 5) << 0) & 63);
   TxData[2] = (TxData[2] & ~248) | ((Inv_R_TempMotor << 3) & 248); 
   TxData[3] = (TxData[3] & ~7) | (((current_R >> 8) << 0) & 7);
   TxData[4] = current_R >> 0;
   TxData[0] = (TxData[0] & ~255) | (((Inv_R_TempIGBT >> 2) << 0) & 255);
   TxData[1] = (TxData[1] & ~192) | ((Inv_R_TempIGBT << 6) & 192); 
   TxData[2] = (TxData[2] & ~7) | (((RR_Vel_ms_Wheel >> 5) << 0) & 7);
   TxData[3] = (TxData[3] & ~248) | ((RR_Vel_ms_Wheel << 3) & 248); 
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_PROC_ETAS_Inverter_L_Data(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 5;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = PROC_ETAS_Inverter_L_Data_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[1] = (TxData[1] & ~63) | (((Inv_L_TempMotor >> 5) << 0) & 63);
   TxData[2] = (TxData[2] & ~248) | ((Inv_L_TempMotor << 3) & 248); 
   TxData[3] = (TxData[3] & ~7) | (((current_L >> 8) << 0) & 7);
   TxData[4] = current_L >> 0;
   TxData[0] = (TxData[0] & ~255) | (((Inv_L_TempIGBT >> 2) << 0) & 255);
   TxData[1] = (TxData[1] & ~192) | ((Inv_L_TempIGBT << 6) & 192); 
   TxData[2] = (TxData[2] & ~7) | (((RL_Vel_ms_Wheel >> 5) << 0) & 7);
   TxData[3] = (TxData[3] & ~248) | ((RL_Vel_ms_Wheel << 3) & 248); 
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_PROC_ETAS_VDC_Suspe(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 7;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = PROC_ETAS_VDC_Suspe_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[5] = (TxData[5] & ~63) | (((SUSP_R_R >> 8) << 0) & 63);
   TxData[6] = SUSP_R_R >> 0;
   TxData[3] = (TxData[3] & ~15) | (((SUSP_R_L >> 10) << 0) & 15);
   TxData[4] = SUSP_R_L >> 2;
   TxData[5] = (TxData[5] & ~192) | ((SUSP_R_L << 6) & 192); 
   TxData[1] = (TxData[1] & ~3) | (((SUSP_F_R >> 12) << 0) & 3);
   TxData[2] = SUSP_F_R >> 4;
   TxData[3] = (TxData[3] & ~240) | ((SUSP_F_R << 4) & 240); 
   TxData[0] = (TxData[0] & ~255) | (((SUSP_F_L >> 6) << 0) & 255);
   TxData[1] = (TxData[1] & ~252) | ((SUSP_F_L << 2) & 252); 
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_PROC_ETAS_VDC_Tq(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 4;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = PROC_ETAS_VDC_Tq_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[2] = (TxData[2] & ~15) | (((Throttle_Torque >> 6) << 0) & 15);
   TxData[3] = (TxData[3] & ~252) | ((Throttle_Torque << 2) & 252); 
   TxData[1] = (TxData[1] & ~63) | (((Torque_R >> 4) << 0) & 63);
   TxData[2] = (TxData[2] & ~240) | ((Torque_R << 4) & 240); 
   TxData[0] = (TxData[0] & ~255) | (((Torque_L >> 2) << 0) & 255);
   TxData[1] = (TxData[1] & ~192) | ((Torque_L << 6) & 192); 
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_RAW_RECU_Data(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 3;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = RAW_RECU_Data_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[0] = (TxData[0] & ~255) | (((Susp_R_R_Bits >> 4) << 0) & 255);
   TxData[1] = (TxData[1] & ~240) | ((Susp_R_R_Bits << 4) & 240); 
   TxData[1] = (TxData[1] & ~15) | (((Susp_R_L_Bits >> 8) << 0) & 15);
   TxData[2] = Susp_R_L_Bits >> 0;
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_STAT_FECU_Keep_Alive(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 7;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = STAT_FECU_Keep_Alive_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[4] = (TxData[4] & ~31) | (((FECU_LV >> 7) << 0) & 31);
   TxData[5] = (TxData[5] & ~254) | ((FECU_LV << 1) & 254); 
   TxData[2] = (TxData[2] & ~1) | (((FECU_5V >> 11) << 0) & 1);
   TxData[3] = FECU_5V >> 3;
   TxData[4] = (TxData[4] & ~224) | ((FECU_5V << 5) & 224); 
   TxData[1] = (TxData[1] & ~31) | (((FECU_3V3 >> 7) << 0) & 31);
   TxData[2] = (TxData[2] & ~254) | ((FECU_3V3 << 1) & 254); 
   TxData[1] = (TxData[1] & ~128) | (((Shutdown_Setas >> 0) << 7) & 128);
   TxData[1] = (TxData[1] & ~32) | (((Shutdown_SC_BOTS >> 0) << 5) & 32);
   TxData[1] = (TxData[1] & ~64) | (((Shutdown_BSPD_Inertia >> 0) << 6) & 64);
   TxData[0] = (TxData[0] & ~255) | (((Front_Alive >> 0) << 0) & 255);
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_STAT_RECU_Keep_Alive(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 7;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = STAT_RECU_Keep_Alive_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[4] = (TxData[4] & ~7) | (((RECU_LV >> 9) << 0) & 7);
   TxData[5] = RECU_LV >> 1;
   TxData[6] = (TxData[6] & ~128) | ((RECU_LV << 7) & 128); 
   TxData[3] = (TxData[3] & ~127) | (((RECU_5V >> 5) << 0) & 127);
   TxData[4] = (TxData[4] & ~248) | ((RECU_5V << 3) & 248); 
   TxData[1] = (TxData[1] & ~7) | (((RECU_3V3 >> 9) << 0) & 7);
   TxData[2] = RECU_3V3 >> 1;
   TxData[3] = (TxData[3] & ~128) | ((RECU_3V3 << 7) & 128); 
   TxData[1] = (TxData[1] & ~128) | (((Shutdown_TSMS_TSMP >> 0) << 7) & 128);
   TxData[1] = (TxData[1] & ~64) | (((Shutdown_RightTS >> 0) << 6) & 64);
   TxData[1] = (TxData[1] & ~32) | (((Shutdown_LeftTS >> 0) << 5) & 32);
   TxData[1] = (TxData[1] & ~8) | (((Shutdown_HVD >> 0) << 3) & 8);
   TxData[1] = (TxData[1] & ~16) | (((Shutdown_HVBox >> 0) << 4) & 16);
   TxData[0] = (TxData[0] & ~255) | (((Rear_Alive >> 0) << 0) & 255);
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_RAW_FECU_Data2(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 8;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = RAW_FECU_Data2_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[6] = (TxData[6] & ~255) | (((BrakePressure2_Bits >> 4) << 0) & 255);
   TxData[7] = (TxData[7] & ~240) | ((BrakePressure2_Bits << 4) & 240); 
   TxData[4] = (TxData[4] & ~15) | (((BrakePressure1_Bits >> 8) << 0) & 15);
   TxData[5] = BrakePressure1_Bits >> 0;
   TxData[3] = (TxData[3] & ~255) | (((Pitot_Bits >> 4) << 0) & 255);
   TxData[4] = (TxData[4] & ~240) | ((Pitot_Bits << 4) & 240); 
   TxData[0] = (TxData[0] & ~255) | (((Susp_F_R_Bits >> 4) << 0) & 255);
   TxData[1] = (TxData[1] & ~240) | ((Susp_F_R_Bits << 4) & 240); 
   TxData[1] = (TxData[1] & ~15) | (((Susp_F_L_Bits >> 8) << 0) & 15);
   TxData[2] = Susp_F_L_Bits >> 0;
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_RAW_FECU_Data1(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 6;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = RAW_FECU_Data1_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[4] = (TxData[4] & ~15) | (((SteeringSensor_Bits >> 8) << 0) & 15);
   TxData[5] = SteeringSensor_Bits >> 0;
   TxData[3] = (TxData[3] & ~255) | (((BrakePedal_Bits >> 4) << 0) & 255);
   TxData[4] = (TxData[4] & ~240) | ((BrakePedal_Bits << 4) & 240); 
   TxData[1] = (TxData[1] & ~15) | (((APPS2_Bits >> 8) << 0) & 15);
   TxData[2] = APPS2_Bits >> 0;
   TxData[0] = (TxData[0] & ~255) | (((APPS1_Bits >> 4) << 0) & 255);
   TxData[1] = (TxData[1] & ~240) | ((APPS1_Bits << 4) & 240); 
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

void message_cantx_STAT_ETAS_Sync(CAN_HandleTypeDef hcan) {
   TxHeader.DLC = 1;
   TxHeader.IDE = 0;
   TxHeader.RTR = 0;
   TxHeader.StdId = STAT_ETAS_Sync_id;
   TxHeader.TransmitGlobalTime = DISABLE;
   TxData[0] = (TxData[0] & ~255) | (((ETAS_Sync_X >> 0) << 0) & 255);
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
        error_count++;
    } else if (error_count > 0){
        error_count--; 
    }
}

//Rx functions-------------------------------------------------------------------------------------------

void message_canrx_VECTOR__INDEPENDENT_SIG_MSG(uint8_t *RxData) {
   ETAS_Warning = ((RxData[0] & 1) << 1) | ((RxData[1] & 128) >> 7);   // Signal: ETAS_Warning Start Bit: 0, Length: 2 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   el_AUTO_STATUS = ((RxData[0] & 1) << 7) | ((RxData[1] & 254) >> 1);   // Signal: el_AUTO_STATUS Start Bit: 0, Length: 8 Byte Order: 1, Value Type: + Factor: 1 Offset: 0 Unit: 
   Inv_R_Icommand = ((RxData[0] & 1) << 7) | ((RxData[1] & 254) >> 1);   // Signal: Inv_R_Icommand Start Bit: 0, Length: 8 Byte Order: 0, Value Type: - Factor: 2.5 Offset: 0 Unit: A
   Inv_R_Iactual = ((RxData[0] & 1) << 7) | ((RxData[1] & 254) >> 1);   // Signal: Inv_R_Iactual Start Bit: 0, Length: 8 Byte Order: 0, Value Type: - Factor: 2.5 Offset: 0 Unit: A
   Inv_L_Icommand = ((RxData[0] & 1) << 7) | ((RxData[1] & 254) >> 1);   // Signal: Inv_L_Icommand Start Bit: 0, Length: 8 Byte Order: 0, Value Type: - Factor: 2.5 Offset: 0 Unit: A
   Inv_L_Iactual = ((RxData[0] & 1) << 7) | ((RxData[1] & 254) >> 1);   // Signal: Inv_L_Iactual Start Bit: 0, Length: 8 Byte Order: 0, Value Type: - Factor: 2.5 Offset: 0 Unit: A
   SOE = ((RxData[0] & 1) << 9) | (RxData[1] << 1) | ((RxData[2] & 128) >> 7);   // Signal: SOE Start Bit: 0, Length: 10 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 0 Unit: %
   ETAS_MSG_Counter = ((RxData[0] & 1) << 15) | (RxData[1] << 7) | ((RxData[2] & 254) >> 1);   // Signal: ETAS_MSG_Counter Start Bit: 0, Length: 16 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: -
   Highest_CellTemp = ((RxData[0] & 1) << 9) | (RxData[1] << 1) | ((RxData[2] & 128) >> 7);   // Signal: Highest_CellTemp Start Bit: 0, Length: 10 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 0 Unit: �C
   SOC_High = ((RxData[0] & 1) << 9) | (RxData[1] << 1) | ((RxData[2] & 128) >> 7);   // Signal: SOC_High Start Bit: 0, Length: 10 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 0 Unit: %
   Lowest_CellVoltage = ((RxData[0] & 1) << 15) | (RxData[1] << 7) | ((RxData[2] & 254) >> 1);   // Signal: Lowest_CellVoltage Start Bit: 0, Length: 16 Byte Order: 0, Value Type: + Factor: 0.0001 Offset: 0 Unit: V
   DeltaSOC_LastLap = ((RxData[0] & 1) << 6) | ((RxData[1] & 252) >> 2);   // Signal: DeltaSOC_LastLap Start Bit: 0, Length: 7 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 2 Unit: %
   Accu_Current = ((RxData[0] & 1) << 31) | (RxData[1] << 23) | (RxData[2] << 15) | (RxData[3] << 7) | ((RxData[4] & 254) >> 1);   // Signal: Accu_Current Start Bit: 0, Length: 32 Byte Order: 0, Value Type: + Factor: -0.001 Offset: 2147483.648 Unit: A
   Reg_Level = ((RxData[0] & 1) << 1) | ((RxData[1] & 128) >> 7);   // Signal: Reg_Level Start Bit: 0, Length: 2 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   BB_Dash = ((RxData[0] & 1) << 5) | ((RxData[1] & 248) >> 3);   // Signal: BB_Dash Start Bit: 0, Length: 6 Byte Order: 0, Value Type: + Factor: 0.5 Offset: 60 Unit: Bit
   Disable_Regen = ((RxData[0] & 1) >> 0);   // Signal: Disable_Regen Start Bit: 0, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Sensorics_Mode = ((RxData[0] & 1) << 1) | ((RxData[1] & 128) >> 7);   // Signal: Sensorics_Mode Start Bit: 0, Length: 2 Byte Order: 0, Value Type: + Factor: 1 Offset: 1 Unit: Bit
   tel_DeltaSOC_LastLap = ((RxData[0] & 1) << 6) | ((RxData[1] & 252) >> 2);   // Signal: tel_DeltaSOC_LastLap Start Bit: 0, Length: 7 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 2 Unit: %
   AvgVEL_LastLap = ((RxData[0] & 1) << 9) | (RxData[1] << 1) | ((RxData[2] & 128) >> 7);   // Signal: AvgVEL_LastLap Start Bit: 0, Length: 10 Byte Order: 0, Value Type: - Factor: 0.1 Offset: 0 Unit: km/h
}

void message_canrx_PROC_ETAS_VDC_Pressure(uint8_t *RxData) {
   BrakePressure2 = ((RxData[1] & 127) << 2) | ((RxData[2] & 192) >> 6);   // Signal: BrakePressure2 Start Bit: 14, Length: 9 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 0 Unit: bar
   BrakePressure1 = ((RxData[0] & 255) << 1) | ((RxData[1] & 128) >> 7);   // Signal: BrakePressure1 Start Bit: 7, Length: 9 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 0 Unit: bar
}

void message_canrx_CAB300(uint8_t *RxData) {
   Accu_Current = ((RxData[0] & 255) << 24) | (RxData[1] << 16) | (RxData[2] << 8) | (RxData[3] << 0);   // Signal: Accu_Current Start Bit: 7, Length: 32 Byte Order: 0, Value Type: + Factor: -0.001 Offset: 2147483.648 Unit: A
}

void message_canrx_PROC_ETAS_Warning(uint8_t *RxData) {
   Warning_Low_Voltage = ((RxData[0] & 64) >> 6);   // Signal: Warning_Low_Voltage Start Bit: 6, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Warning_Inv_Temp = ((RxData[0] & 32) >> 5);   // Signal: Warning_Inv_Temp Start Bit: 5, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Warning_Mot_Temp = ((RxData[0] & 16) >> 4);   // Signal: Warning_Mot_Temp Start Bit: 4, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Warning_Accu_Temp = ((RxData[0] & 128) >> 7);   // Signal: Warning_Accu_Temp Start Bit: 7, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
}

void message_canrx_TEL_ETAS_Accu(uint8_t *RxData) {
   tel_VoltageCell_Lowest = ((RxData[2] & 7) << 5) | ((RxData[3] & 248) >> 3);   // Signal: tel_VoltageCell_Lowest Start Bit: 18, Length: 8 Byte Order: 0, Value Type: + Factor: 0.01 Offset: 2 Unit: V
   tel_AccuVoltage = ((RxData[1] & 63) << 5) | ((RxData[2] & 248) >> 3);   // Signal: tel_AccuVoltage Start Bit: 13, Length: 11 Byte Order: 0, Value Type: + Factor: 0.13 Offset: 340 Unit: V
   tel_SOC_Low = ((RxData[0] & 255) << 2) | ((RxData[1] & 192) >> 6);   // Signal: tel_SOC_Low Start Bit: 7, Length: 10 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 0 Unit: %
}

void message_canrx_TEL_ETAS_Info(uint8_t *RxData) {
   tel_IqR = ((RxData[1] & 255) >> 0);   // Signal: tel_IqR Start Bit: 15, Length: 8 Byte Order: 0, Value Type: - Factor: 1.5 Offset: 0 Unit: A
   tel_IqL = ((RxData[2] & 255) >> 0);   // Signal: tel_IqL Start Bit: 23, Length: 8 Byte Order: 0, Value Type: - Factor: 1.5 Offset: 0 Unit: A
   tel_VEL = ((RxData[0] & 255) >> 0);   // Signal: tel_VEL Start Bit: 7, Length: 8 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 0 Unit: m/s
}

void message_canrx_TEL_ETAS_Temps(uint8_t *RxData) {
   tel_TempCell_Highest = ((RxData[5] & 63) << 4) | ((RxData[6] & 240) >> 4);   // Signal: tel_TempCell_Highest Start Bit: 45, Length: 10 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 0 Unit: �C
   tel_TempMotor_R = ((RxData[3] & 1) << 10) | (RxData[4] << 2) | ((RxData[5] & 192) >> 6);   // Signal: tel_TempMotor_R Start Bit: 24, Length: 11 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 0 Unit: �C
   tel_TempIGBT_R = ((RxData[2] & 7) << 7) | ((RxData[3] & 254) >> 1);   // Signal: tel_TempIGBT_R Start Bit: 18, Length: 10 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 0 Unit: �C
   tel_TempMotor_L = ((RxData[1] & 63) << 5) | ((RxData[2] & 248) >> 3);   // Signal: tel_TempMotor_L Start Bit: 13, Length: 11 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 0 Unit: �C
   tel_TempIGBT_L = ((RxData[0] & 255) << 2) | ((RxData[1] & 192) >> 6);   // Signal: tel_TempIGBT_L Start Bit: 7, Length: 10 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 0 Unit: �C
}

void message_canrx_NM_BMS_Charger_Keep_Alive(uint8_t *RxData) {
   BMS_Alive = ((RxData[0] & 255) >> 0);   // Signal: BMS_Alive Start Bit: 7, Length: 8 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: -
}

void message_canrx_STAT_BMS_SFR(uint8_t *RxData) {
   BMS_Balancing_Enable = ((RxData[2] & 64) >> 6);   // Signal: BMS_Balancing_Enable Start Bit: 22, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   BMS_Charge_Flag = ((RxData[2] & 128) >> 7);   // Signal: BMS_Charge_Flag Start Bit: 23, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   BMS_OK = ((RxData[1] & 8) >> 3);   // Signal: BMS_OK Start Bit: 11, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   BMS_CAN_Disconnection = ((RxData[1] & 16) >> 4);   // Signal: BMS_CAN_Disconnection Start Bit: 12, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   BMS_Voltage_Disconnection = ((RxData[1] & 32) >> 5);   // Signal: BMS_Voltage_Disconnection Start Bit: 13, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   BMS_NTC_Disconnection = ((RxData[1] & 64) >> 6);   // Signal: BMS_NTC_Disconnection Start Bit: 14, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   BMS_Current_Disconnection = ((RxData[1] & 128) >> 7);   // Signal: BMS_Current_Disconnection Start Bit: 15, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   BMS_Undertemperature = ((RxData[0] & 1) >> 0);   // Signal: BMS_Undertemperature Start Bit: 0, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   BMS_Overtemperature = ((RxData[0] & 2) >> 1);   // Signal: BMS_Overtemperature Start Bit: 1, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   BMS_Overvoltage = ((RxData[0] & 4) >> 2);   // Signal: BMS_Overvoltage Start Bit: 2, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   BMS_Undervoltage = ((RxData[0] & 8) >> 3);   // Signal: BMS_Undervoltage Start Bit: 3, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Precharge_State = ((RxData[0] & 16) >> 4);   // Signal: Precharge_State Start Bit: 4, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   AIR_Plus_State = ((RxData[0] & 32) >> 5);   // Signal: AIR_Plus_State Start Bit: 5, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   AIR_Minus_State = ((RxData[0] & 64) >> 6);   // Signal: AIR_Minus_State Start Bit: 6, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Shutdown_PackageIntck = ((RxData[0] & 128) >> 7);   // Signal: Shutdown_PackageIntck Start Bit: 7, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
}

void message_canrx_CHARGER_Battery_Current(uint8_t *RxData) {
   Charger_Accumulator_Current = ((RxData[0] & 255) << 16) | (RxData[1] << 8) | (RxData[2] << 0);   // Signal: Charger_Accumulator_Current Start Bit: 7, Length: 24 Byte Order: 0, Value Type: + Factor: 0.001 Offset: 0 Unit: 
}

void message_canrx_STAT_CHARGER_Battery_Temp(uint8_t *RxData) {
   Charger_Lowest_Cell_Temperature = ((RxData[0] & 255) << 8) | (RxData[1] << 0);   // Signal: Charger_Lowest_Cell_Temperature Start Bit: 7, Length: 16 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 0 Unit: 
   Charger_Highest_Cell_Temperature = ((RxData[2] & 255) << 8) | (RxData[3] << 0);   // Signal: Charger_Highest_Cell_Temperature Start Bit: 23, Length: 16 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 0 Unit: 
   Charger_Average_Cell_Temperature = ((RxData[4] & 255) << 8) | (RxData[5] << 0);   // Signal: Charger_Average_Cell_Temperature Start Bit: 39, Length: 16 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 0 Unit: 
}

void message_canrx_STAT_CHARGER_Battery_Voltages(uint8_t *RxData) {
   Charger_Acumulator_Voltage = ((RxData[4] & 255) << 16) | (RxData[5] << 8) | (RxData[6] << 0);   // Signal: Charger_Acumulator_Voltage Start Bit: 39, Length: 24 Byte Order: 0, Value Type: + Factor: 0.001 Offset: 0 Unit: 
   Charger_Highest_Cell_Voltage = ((RxData[2] & 255) << 8) | (RxData[3] << 0);   // Signal: Charger_Highest_Cell_Voltage Start Bit: 23, Length: 16 Byte Order: 0, Value Type: + Factor: 0.001 Offset: 0 Unit: 
   Charger_Lowest_Cell_Voltage = ((RxData[0] & 255) << 8) | (RxData[1] << 0);   // Signal: Charger_Lowest_Cell_Voltage Start Bit: 7, Length: 16 Byte Order: 0, Value Type: + Factor: 0.001 Offset: 0 Unit: 
}

void message_canrx_STAT_CHARGER_PrechargeOK(uint8_t *RxData) {
   Charger_PrechargeOK = ((RxData[0] & 255) >> 0);   // Signal: Charger_PrechargeOK Start Bit: 7, Length: 8 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: -
}

void message_canrx_STAT_BMS_AIRs_State(uint8_t *RxData) {
   Charger_AIRs_State = ((RxData[0] & 15) >> 0);   // Signal: Charger_AIRs_State Start Bit: 3, Length: 4 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: -
}

void message_canrx_CTRL_CHARGER_AIRs_Request(uint8_t *RxData) {
   Charger_AIRs_Request = ((RxData[0] & 15) >> 0);   // Signal: Charger_AIRs_Request Start Bit: 3, Length: 4 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: -
}

void message_canrx_STAT_CHARGER_Sync(uint8_t *RxData) {
   Charger_Sync = ((RxData[0] & 255) >> 0);   // Signal: Charger_Sync Start Bit: 7, Length: 8 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: -
}

void message_canrx_PROC_ETAS_VDC_Params(uint8_t *RxData) {
   VDC_RB_TqD = ((RxData[7] & 240) >> 4);   // Signal: VDC_RB_TqD Start Bit: 63, Length: 4 Byte Order: 0, Value Type: + Factor: -1 Offset: 0 Unit: Nm
   VDC_RB_TqBP = ((RxData[7] & 15) >> 0);   // Signal: VDC_RB_TqBP Start Bit: 59, Length: 4 Byte Order: 0, Value Type: + Factor: -1 Offset: 0 Unit: Nm
   VDC_BP_SatUp = ((RxData[6] & 63) >> 0);   // Signal: VDC_BP_SatUp Start Bit: 53, Length: 6 Byte Order: 0, Value Type: + Factor: 0.01 Offset: 0.37 Unit: 1%
   VDC_BP_SatDown = ((RxData[5] & 15) << 2) | ((RxData[6] & 192) >> 6);   // Signal: VDC_BP_SatDown Start Bit: 43, Length: 6 Byte Order: 0, Value Type: + Factor: 0.01 Offset: 0 Unit: 1%
   VDC_AP_SatRB = ((RxData[4] & 3) << 4) | ((RxData[5] & 240) >> 4);   // Signal: VDC_AP_SatRB Start Bit: 33, Length: 6 Byte Order: 0, Value Type: + Factor: 0.01 Offset: 0 Unit: 1%
   VDC_Steering_Deadzone = ((RxData[3] & 248) >> 3);   // Signal: VDC_Steering_Deadzone Start Bit: 31, Length: 5 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: �
   VDC_Min_Tyre_Slip = ((RxData[0] & 3) << 4) | ((RxData[1] & 240) >> 4);   // Signal: VDC_Min_Tyre_Slip Start Bit: 1, Length: 6 Byte Order: 0, Value Type: + Factor: 0.01 Offset: 1 Unit: -
   VDC_Max_Tyre_Slip = ((RxData[0] & 252) >> 2);   // Signal: VDC_Max_Tyre_Slip Start Bit: 7, Length: 6 Byte Order: 0, Value Type: + Factor: 0.01 Offset: 1 Unit: -
   VDC_Max_TV_DiffTq = ((RxData[4] & 124) >> 2);   // Signal: VDC_Max_TV_DiffTq Start Bit: 38, Length: 5 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Nm
   VDC_Max_Steering_Angle = ((RxData[3] & 7) << 1) | ((RxData[4] & 128) >> 7);   // Signal: VDC_Max_Steering_Angle Start Bit: 26, Length: 4 Byte Order: 0, Value Type: + Factor: 1 Offset: 115 Unit: �
   VDC_AP_SatUp = ((RxData[2] & 63) >> 0);   // Signal: VDC_AP_SatUp Start Bit: 21, Length: 6 Byte Order: 0, Value Type: + Factor: 0.01 Offset: 0.37 Unit: 1%
   VDC_AP_SatDown = ((RxData[1] & 15) << 2) | ((RxData[2] & 192) >> 6);   // Signal: VDC_AP_SatDown Start Bit: 11, Length: 6 Byte Order: 0, Value Type: + Factor: 0.01 Offset: 0 Unit: 1%
}

void message_canrx_PROC_ETAS_VDC_LapTiming(uint8_t *RxData) {
   RemainLaps = ((RxData[5] & 62) >> 1);   // Signal: RemainLaps Start Bit: 45, Length: 5 Byte Order: 0, Value Type: + Factor: 1 Offset: 1 Unit: -
   DeltaSOC_LastLap = ((RxData[4] & 31) << 2) | ((RxData[5] & 192) >> 6);   // Signal: DeltaSOC_LastLap Start Bit: 36, Length: 7 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 2 Unit: %
   AvgVEL_LastLap = ((RxData[3] & 31) << 3) | ((RxData[4] & 224) >> 5);   // Signal: AvgVEL_LastLap Start Bit: 28, Length: 8 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 23 Unit: km/h
   TotalTime = ((RxData[2] & 255) << 3) | ((RxData[3] & 224) >> 5);   // Signal: TotalTime Start Bit: 23, Length: 11 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: s
   LapCount = ((RxData[1] & 31) >> 0);   // Signal: LapCount Start Bit: 12, Length: 5 Byte Order: 0, Value Type: + Factor: 1 Offset: 1 Unit: -
   LapTime = ((RxData[0] & 255) << 3) | ((RxData[1] & 224) >> 5);   // Signal: LapTime Start Bit: 7, Length: 11 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 0 Unit: s
}

void message_canrx_CTRL_BMS_Accu_Data(uint8_t *RxData) {
   Average_CellTemp = ((RxData[4] & 3) << 8) | (RxData[5] << 0);   // Signal: Average_CellTemp Start Bit: 33, Length: 10 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 0 Unit: �C
   Accumulator_Voltage = ((RxData[0] & 7) << 18) | (RxData[1] << 10) | (RxData[2] << 2) | ((RxData[3] & 192) >> 6);   // Signal: Accumulator_Voltage Start Bit: 2, Length: 21 Byte Order: 0, Value Type: + Factor: 0.00015 Offset: 340 Unit: V
   Accumulator_Current = ((RxData[3] & 63) << 6) | ((RxData[4] & 252) >> 2);   // Signal: Accumulator_Current Start Bit: 29, Length: 12 Byte Order: 0, Value Type: + Factor: 0.19536 Offset: -400 Unit: A
   Shutdown_PackageIntck = ((RxData[0] & 32) >> 5);   // Signal: Shutdown_PackageIntck Start Bit: 5, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Shutdown_IMD = ((RxData[0] & 8) >> 3);   // Signal: Shutdown_IMD Start Bit: 3, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Shutdown_BMS = ((RxData[0] & 16) >> 4);   // Signal: Shutdown_BMS Start Bit: 4, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   AIRs_State = ((RxData[0] & 192) >> 6);   // Signal: AIRs_State Start Bit: 7, Length: 2 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: -
}

void message_canrx_CTRL_BMS_Cell_Extremes(uint8_t *RxData) {
   Highest_CellVoltage = ((RxData[2] & 255) << 8) | (RxData[3] << 0);   // Signal: Highest_CellVoltage Start Bit: 23, Length: 16 Byte Order: 0, Value Type: + Factor: 0.0001 Offset: 0 Unit: V
   Highest_CellTemp = ((RxData[5] & 63) << 4) | ((RxData[6] & 240) >> 4);   // Signal: Highest_CellTemp Start Bit: 45, Length: 10 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 0 Unit: �C
   Lowest_CellVoltage = ((RxData[0] & 255) << 8) | (RxData[1] << 0);   // Signal: Lowest_CellVoltage Start Bit: 7, Length: 16 Byte Order: 0, Value Type: + Factor: 0.0001 Offset: 0 Unit: V
   Lowest_CellTemp = ((RxData[4] & 255) << 2) | ((RxData[5] & 192) >> 6);   // Signal: Lowest_CellTemp Start Bit: 39, Length: 10 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 0 Unit: �C
}

void message_canrx_STAT_DASH_Keep_Alive(uint8_t *RxData) {
   DASH_LV = ((RxData[4] & 255) << 4) | ((RxData[5] & 240) >> 4);   // Signal: DASH_LV Start Bit: 39, Length: 12 Byte Order: 0, Value Type: + Factor: 0.008865 Offset: 0 Unit: V
   DASH_5V = ((RxData[2] & 15) << 8) | (RxData[3] << 0);   // Signal: DASH_5V Start Bit: 19, Length: 12 Byte Order: 0, Value Type: + Factor: 0.00241758 Offset: 0 Unit: V
   DASH_3V3 = ((RxData[1] & 255) << 4) | ((RxData[2] & 240) >> 4);   // Signal: DASH_3V3 Start Bit: 15, Length: 12 Byte Order: 0, Value Type: + Factor: 0.00080586 Offset: 0 Unit: V
   Dash_Alive = ((RxData[0] & 255) >> 0);   // Signal: Dash_Alive Start Bit: 7, Length: 8 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: -
}

void message_canrx_STAT_BMS_Keep_Alive(uint8_t *RxData) {
   IMD_OK = ((RxData[5] & 4) >> 2);   // Signal: IMD_OK Start Bit: 42, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: 
   BMS_OK = ((RxData[5] & 8) >> 3);   // Signal: BMS_OK Start Bit: 43, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: 
   BMS_LV = ((RxData[4] & 255) << 4) | ((RxData[5] & 240) >> 4);   // Signal: BMS_LV Start Bit: 39, Length: 12 Byte Order: 0, Value Type: + Factor: 0.008865 Offset: 0 Unit: V
   BMS_5V = ((RxData[2] & 15) << 8) | (RxData[3] << 0);   // Signal: BMS_5V Start Bit: 19, Length: 12 Byte Order: 0, Value Type: + Factor: 0.001394 Offset: 0 Unit: V
   BMS_12V = ((RxData[1] & 255) << 4) | ((RxData[2] & 240) >> 4);   // Signal: BMS_12V Start Bit: 15, Length: 12 Byte Order: 0, Value Type: + Factor: 0.00459 Offset: 0 Unit: V
   BMS_Alive = ((RxData[0] & 255) >> 0);   // Signal: BMS_Alive Start Bit: 7, Length: 8 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: -
}

void message_canrx_PROC_ETAS_TS_Data(uint8_t *RxData) {
   Precharge_Percentage = ((RxData[5] & 15) << 3) | ((RxData[6] & 224) >> 5);   // Signal: Precharge_Percentage Start Bit: 43, Length: 7 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: %
   Power_Accu = ((RxData[3] & 3) << 12) | (RxData[4] << 4) | ((RxData[5] & 240) >> 4);   // Signal: Power_Accu Start Bit: 25, Length: 14 Byte Order: 0, Value Type: - Factor: 0.01 Offset: 5 Unit: kW
   SOC_High = ((RxData[2] & 15) << 6) | ((RxData[3] & 252) >> 2);   // Signal: SOC_High Start Bit: 19, Length: 10 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 0 Unit: %
   SOC_Low = ((RxData[1] & 63) << 4) | ((RxData[2] & 240) >> 4);   // Signal: SOC_Low Start Bit: 13, Length: 10 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 0 Unit: %
   SOC_Avg = ((RxData[0] & 255) << 2) | ((RxData[1] & 192) >> 6);   // Signal: SOC_Avg Start Bit: 7, Length: 10 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 0 Unit: %
}

void message_canrx_PROC_ELLIPSE_Auto(uint8_t *RxData) {
   el_Track_Valid = ((RxData[6] & 1) >> 0);   // Signal: el_Track_Valid Start Bit: 48, Length: 1 Byte Order: 1, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   el_Slip_Valid = ((RxData[6] & 2) >> 1);   // Signal: el_Slip_Valid Start Bit: 49, Length: 1 Byte Order: 1, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   el_Curvature_Valid = ((RxData[6] & 4) >> 2);   // Signal: el_Curvature_Valid Start Bit: 50, Length: 1 Byte Order: 1, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   el_SlipAngle = ((RxData[2] & 1) << 15) | (RxData[3] << 7) | ((RxData[4] & 254) >> 1);   // Signal: el_SlipAngle Start Bit: 16, Length: 16 Byte Order: 1, Value Type: - Factor: 0.0001 Offset: 0 Unit: rad
   el_CurvatureRadius = ((RxData[4] & 1) << 15) | (RxData[5] << 7) | ((RxData[6] & 254) >> 1);   // Signal: el_CurvatureRadius Start Bit: 32, Length: 16 Byte Order: 1, Value Type: + Factor: 0.01 Offset: 0 Unit: m
   el_AngleTrack = ((RxData[0] & 1) << 15) | (RxData[1] << 7) | ((RxData[2] & 254) >> 1);   // Signal: el_AngleTrack Start Bit: 0, Length: 16 Byte Order: 1, Value Type: - Factor: 0.0001 Offset: 0 Unit: rad
}

void message_canrx_CTRL_ELLIPSE_GPS_Vel(uint8_t *RxData) {
   el_Vel_GPS_N = ((RxData[0] & 1) << 15) | (RxData[1] << 7) | ((RxData[2] & 254) >> 1);   // Signal: el_Vel_GPS_N Start Bit: 0, Length: 16 Byte Order: 1, Value Type: - Factor: 0.01 Offset: 0 Unit: m/s
   el_Vel_GPS_E = ((RxData[2] & 1) << 15) | (RxData[3] << 7) | ((RxData[4] & 254) >> 1);   // Signal: el_Vel_GPS_E Start Bit: 16, Length: 16 Byte Order: 1, Value Type: - Factor: 0.01 Offset: 0 Unit: m/s
   el_Vel_GPS_D = ((RxData[4] & 1) << 15) | (RxData[5] << 7) | ((RxData[6] & 254) >> 1);   // Signal: el_Vel_GPS_D Start Bit: 32, Length: 16 Byte Order: 1, Value Type: - Factor: 0.01 Offset: 0 Unit: m/s
}

void message_canrx_STAT_ETAS_Diagnostics(uint8_t *RxData) {
   InvertersAction = ((RxData[3] & 14) >> 1);   // Signal: InvertersAction Start Bit: 27, Length: 3 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: -
   Relay_Error = ((RxData[3] & 64) >> 6);   // Signal: Relay_Error Start Bit: 30, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Regen_Enabled = ((RxData[3] & 128) >> 7);   // Signal: Regen_Enabled Start Bit: 31, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   TC_Warning = ((RxData[2] & 1) >> 0);   // Signal: TC_Warning Start Bit: 16, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Sensorics_Mode = ((RxData[3] & 48) >> 4);   // Signal: Sensorics_Mode Start Bit: 29, Length: 2 Byte Order: 0, Value Type: + Factor: 1 Offset: 1 Unit: -
   el_Vel_OK = ((RxData[2] & 2) >> 1);   // Signal: el_Vel_OK Start Bit: 17, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Torque_OK = ((RxData[2] & 4) >> 2);   // Signal: Torque_OK Start Bit: 18, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   APPS_Implausibility = ((RxData[2] & 8) >> 3);   // Signal: APPS_Implausibility Start Bit: 19, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Disconnection_BrakePressure2 = ((RxData[2] & 16) >> 4);   // Signal: Disconnection_BrakePressure2 Start Bit: 20, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Disconnection_BrakePressure1 = ((RxData[2] & 32) >> 5);   // Signal: Disconnection_BrakePressure1 Start Bit: 21, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Critical_Signal_Disconnection = ((RxData[0] & 128) >> 7);   // Signal: Critical_Signal_Disconnection Start Bit: 7, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Critical_CAN_Disconnection = ((RxData[0] & 64) >> 6);   // Signal: Critical_CAN_Disconnection Start Bit: 6, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Disconnection_InvR = ((RxData[2] & 64) >> 6);   // Signal: Disconnection_InvR Start Bit: 22, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Disconnection_InvL = ((RxData[2] & 128) >> 7);   // Signal: Disconnection_InvL Start Bit: 23, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Disconnection_Susp_R_R = ((RxData[1] & 16) >> 4);   // Signal: Disconnection_Susp_R_R Start Bit: 12, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Disconnection_Susp_R_L = ((RxData[1] & 8) >> 3);   // Signal: Disconnection_Susp_R_L Start Bit: 11, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Disconnection_Susp_F_R = ((RxData[1] & 4) >> 2);   // Signal: Disconnection_Susp_F_R Start Bit: 10, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Disconnection_Susp_F_L = ((RxData[1] & 2) >> 1);   // Signal: Disconnection_Susp_F_L Start Bit: 9, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Disconnection_SteeringSensor = ((RxData[1] & 32) >> 5);   // Signal: Disconnection_SteeringSensor Start Bit: 13, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Disconnection_Rear = ((RxData[0] & 2) >> 1);   // Signal: Disconnection_Rear Start Bit: 1, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Disconnection_Pitot = ((RxData[1] & 1) >> 0);   // Signal: Disconnection_Pitot Start Bit: 8, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Disconnection_Front = ((RxData[0] & 8) >> 3);   // Signal: Disconnection_Front Start Bit: 3, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Disconnection_Ellipse = ((RxData[0] & 4) >> 2);   // Signal: Disconnection_Ellipse Start Bit: 2, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Disconnection_DashBoard = ((RxData[0] & 16) >> 4);   // Signal: Disconnection_DashBoard Start Bit: 4, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Disconnection_BrakePedal = ((RxData[1] & 64) >> 6);   // Signal: Disconnection_BrakePedal Start Bit: 14, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Disconnection_BMS = ((RxData[0] & 32) >> 5);   // Signal: Disconnection_BMS Start Bit: 5, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Disconnection_APPS2 = ((RxData[1] & 128) >> 7);   // Signal: Disconnection_APPS2 Start Bit: 15, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Disconnection_APPS1 = ((RxData[0] & 1) >> 0);   // Signal: Disconnection_APPS1 Start Bit: 0, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
}

void message_canrx_PROC_ETAS_VDC_Values(uint8_t *RxData) {
   Inv_Speed = ((RxData[5] & 127) >> 0);   // Signal: Inv_Speed Start Bit: 46, Length: 7 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: km/h
   el_VEL = ((RxData[3] & 7) << 9) | (RxData[4] << 1) | ((RxData[5] & 128) >> 7);   // Signal: el_VEL Start Bit: 26, Length: 12 Byte Order: 0, Value Type: + Factor: 0.01 Offset: 0 Unit: m/s
   SteeringSensor_Value = ((RxData[2] & 7) << 5) | ((RxData[3] & 248) >> 3);   // Signal: SteeringSensor_Value Start Bit: 18, Length: 8 Byte Order: 0, Value Type: - Factor: 1 Offset: 0 Unit: �
   BrakePedal_Value = ((RxData[1] & 3) << 5) | ((RxData[2] & 248) >> 3);   // Signal: BrakePedal_Value Start Bit: 9, Length: 7 Byte Order: 0, Value Type: + Factor: 0.01 Offset: 0 Unit: 1%
   APPS2_Value = ((RxData[0] & 1) << 6) | ((RxData[1] & 252) >> 2);   // Signal: APPS2_Value Start Bit: 0, Length: 7 Byte Order: 0, Value Type: + Factor: 0.01 Offset: 0 Unit: 1%
   APPS1_Value = ((RxData[0] & 254) >> 1);   // Signal: APPS1_Value Start Bit: 7, Length: 7 Byte Order: 0, Value Type: + Factor: 0.01 Offset: 0 Unit: 1%
}

void message_canrx_CTRL_DASH_Driver_Inputs(uint8_t *RxData) {
   Refri_ACCU = ((RxData[2] & 3) >> 0);   // Signal: Refri_ACCU Start Bit: 17, Length: 2 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: 
   Pump_R = ((RxData[2] & 16) >> 4);   // Signal: Pump_R Start Bit: 20, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Pump_L = ((RxData[2] & 32) >> 5);   // Signal: Pump_L Start Bit: 21, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Button_2 = ((RxData[2] & 4) >> 2);   // Signal: Button_2 Start Bit: 18, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Button_1 = ((RxData[2] & 8) >> 3);   // Signal: Button_1 Start Bit: 19, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   TV_Level = ((RxData[1] & 28) >> 2);   // Signal: TV_Level Start Bit: 12, Length: 3 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: -
   TC_Level = ((RxData[1] & 224) >> 5);   // Signal: TC_Level Start Bit: 15, Length: 3 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: -
   Driver = ((RxData[0] & 7) >> 0);   // Signal: Driver Start Bit: 2, Length: 3 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: -
   RacingMode = ((RxData[0] & 56) >> 3);   // Signal: RacingMode Start Bit: 5, Length: 3 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: -
   EnableDrive_Order = ((RxData[0] & 64) >> 6);   // Signal: EnableDrive_Order Start Bit: 6, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   PrechargeRequest = ((RxData[0] & 128) >> 7);   // Signal: PrechargeRequest Start Bit: 7, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Fans_R = ((RxData[2] & 192) >> 6);   // Signal: Fans_R Start Bit: 23, Length: 2 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: -
   Fans_L = ((RxData[1] & 3) >> 0);   // Signal: Fans_L Start Bit: 9, Length: 2 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: -
}

void message_canrx_CTRL_ETAS_System(uint8_t *RxData) {
   Car_State = ((RxData[0] & 56) >> 3);   // Signal: Car_State Start Bit: 5, Length: 3 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: -
   Car_OK = ((RxData[0] & 2) >> 1);   // Signal: Car_OK Start Bit: 1, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   AIRs_Request = ((RxData[0] & 192) >> 6);   // Signal: AIRs_Request Start Bit: 7, Length: 2 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: -
   BrakeLight_Control = ((RxData[0] & 4) >> 2);   // Signal: BrakeLight_Control Start Bit: 2, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
}

void message_canrx_PROC_ELLIPSE_EKF_Pos(uint8_t *RxData) {
}

void message_canrx_PROC_ELLIPSE_IMU_Accel(uint8_t *RxData) {
   el_Accel_Z = ((RxData[4] & 1) << 15) | (RxData[5] << 7) | ((RxData[6] & 254) >> 1);   // Signal: el_Accel_Z Start Bit: 32, Length: 16 Byte Order: 1, Value Type: - Factor: 0.01 Offset: 0 Unit: m/s2
   el_Accel_Y = ((RxData[2] & 1) << 15) | (RxData[3] << 7) | ((RxData[4] & 254) >> 1);   // Signal: el_Accel_Y Start Bit: 16, Length: 16 Byte Order: 1, Value Type: - Factor: 0.01 Offset: 0 Unit: m/s2
   el_Accel_X = ((RxData[0] & 1) << 15) | (RxData[1] << 7) | ((RxData[2] & 254) >> 1);   // Signal: el_Accel_X Start Bit: 0, Length: 16 Byte Order: 1, Value Type: - Factor: 0.01 Offset: 0 Unit: m/s2
}

void message_canrx_PROC_ELLIPSE_EKF_Euler(uint8_t *RxData) {
   el_YAW = ((RxData[4] & 1) << 15) | (RxData[5] << 7) | ((RxData[6] & 254) >> 1);   // Signal: el_YAW Start Bit: 32, Length: 16 Byte Order: 1, Value Type: - Factor: 0.0001 Offset: 0 Unit: rad
   el_Pitch = ((RxData[2] & 1) << 15) | (RxData[3] << 7) | ((RxData[4] & 254) >> 1);   // Signal: el_Pitch Start Bit: 16, Length: 16 Byte Order: 1, Value Type: - Factor: 0.0001 Offset: 0 Unit: rad
   el_Roll = ((RxData[0] & 1) << 15) | (RxData[1] << 7) | ((RxData[2] & 254) >> 1);   // Signal: el_Roll Start Bit: 0, Length: 16 Byte Order: 1, Value Type: - Factor: 0.0001 Offset: 0 Unit: rad
}

void message_canrx_CTRL_ELLIPSE_EKF_Vel(uint8_t *RxData) {
   el_Vel_EKF_Z = ((RxData[4] & 1) << 15) | (RxData[5] << 7) | ((RxData[6] & 254) >> 1);   // Signal: el_Vel_EKF_Z Start Bit: 32, Length: 16 Byte Order: 1, Value Type: - Factor: 0.01 Offset: 0 Unit: m/s
   el_Vel_EKF_Y = ((RxData[2] & 1) << 15) | (RxData[3] << 7) | ((RxData[4] & 254) >> 1);   // Signal: el_Vel_EKF_Y Start Bit: 16, Length: 16 Byte Order: 1, Value Type: - Factor: 0.01 Offset: 0 Unit: m/s
   el_Vel_EKF_X = ((RxData[0] & 1) << 15) | (RxData[1] << 7) | ((RxData[2] & 254) >> 1);   // Signal: el_Vel_EKF_X Start Bit: 0, Length: 16 Byte Order: 1, Value Type: - Factor: 0.01 Offset: 0 Unit: m/s
}

void message_canrx_STAT_ELLIPSE_Status(uint8_t *RxData) {
}

void message_canrx_PROC_ELLIPSE_IMU_Gyro(uint8_t *RxData) {
   el_Gyro_Z = ((RxData[4] & 1) << 15) | (RxData[5] << 7) | ((RxData[6] & 254) >> 1);   // Signal: el_Gyro_Z Start Bit: 32, Length: 16 Byte Order: 1, Value Type: - Factor: 0.001 Offset: 0 Unit: rad/s
   el_Gyro_Y = ((RxData[2] & 1) << 15) | (RxData[3] << 7) | ((RxData[4] & 254) >> 1);   // Signal: el_Gyro_Y Start Bit: 16, Length: 16 Byte Order: 1, Value Type: - Factor: 0.001 Offset: 0 Unit: rad/s
   el_Gyro_X = ((RxData[0] & 1) << 15) | (RxData[1] << 7) | ((RxData[2] & 254) >> 1);   // Signal: el_Gyro_X Start Bit: 0, Length: 16 Byte Order: 1, Value Type: - Factor: 0.001 Offset: 0 Unit: rad/s
}

void message_canrx_CTRL_ELLIPSE_Vel_Valid(uint8_t *RxData) {
   el_SolutionMode = ((RxData[0] & 1) << 3) | ((RxData[1] & 224) >> 5);   // Signal: el_SolutionMode Start Bit: 0, Length: 4 Byte Order: 1, Value Type: + Factor: 1 Offset: 0 Unit: -
   el_Vel_GPS_Valid = ((RxData[1] & 4) >> 2);   // Signal: el_Vel_GPS_Valid Start Bit: 10, Length: 1 Byte Order: 1, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   el_Vel_EKF_Valid = ((RxData[0] & 64) >> 6);   // Signal: el_Vel_EKF_Valid Start Bit: 6, Length: 1 Byte Order: 1, Value Type: + Factor: 1 Offset: 0 Unit: Bit
}

void message_canrx_PROC_ETAS_Inverter_R_Data(uint8_t *RxData) {
   Inv_R_TempMotor = ((RxData[1] & 63) << 5) | ((RxData[2] & 248) >> 3);   // Signal: Inv_R_TempMotor Start Bit: 13, Length: 11 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 0 Unit: �C
   current_R = ((RxData[3] & 7) << 8) | (RxData[4] << 0);   // Signal: current_R Start Bit: 26, Length: 11 Byte Order: 0, Value Type: - Factor: 0.1 Offset: 0 Unit: A
   Inv_R_TempIGBT = ((RxData[0] & 255) << 2) | ((RxData[1] & 192) >> 6);   // Signal: Inv_R_TempIGBT Start Bit: 7, Length: 10 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 0 Unit: �C
   RR_Vel_ms_Wheel = ((RxData[2] & 7) << 5) | ((RxData[3] & 248) >> 3);   // Signal: RR_Vel_ms_Wheel Start Bit: 18, Length: 8 Byte Order: 0, Value Type: - Factor: 0.1 Offset: 12 Unit: m/s
}

void message_canrx_PROC_ETAS_Inverter_L_Data(uint8_t *RxData) {
   Inv_L_TempMotor = ((RxData[1] & 63) << 5) | ((RxData[2] & 248) >> 3);   // Signal: Inv_L_TempMotor Start Bit: 13, Length: 11 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 0 Unit: �C
   current_L = ((RxData[3] & 7) << 8) | (RxData[4] << 0);   // Signal: current_L Start Bit: 26, Length: 11 Byte Order: 0, Value Type: - Factor: 0.1 Offset: 0 Unit: A
   Inv_L_TempIGBT = ((RxData[0] & 255) << 2) | ((RxData[1] & 192) >> 6);   // Signal: Inv_L_TempIGBT Start Bit: 7, Length: 10 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 0 Unit: �C
   RL_Vel_ms_Wheel = ((RxData[2] & 7) << 5) | ((RxData[3] & 248) >> 3);   // Signal: RL_Vel_ms_Wheel Start Bit: 18, Length: 8 Byte Order: 0, Value Type: - Factor: 0.1 Offset: 12 Unit: m/s
}

void message_canrx_PROC_ETAS_VDC_Suspe(uint8_t *RxData) {
   SUSP_R_R = ((RxData[5] & 63) << 8) | (RxData[6] << 0);   // Signal: SUSP_R_R Start Bit: 45, Length: 14 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 0 Unit: N
   SUSP_R_L = ((RxData[3] & 15) << 10) | (RxData[4] << 2) | ((RxData[5] & 192) >> 6);   // Signal: SUSP_R_L Start Bit: 27, Length: 14 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 0 Unit: N
   SUSP_F_R = ((RxData[1] & 3) << 12) | (RxData[2] << 4) | ((RxData[3] & 240) >> 4);   // Signal: SUSP_F_R Start Bit: 9, Length: 14 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 0 Unit: N
   SUSP_F_L = ((RxData[0] & 255) << 6) | ((RxData[1] & 252) >> 2);   // Signal: SUSP_F_L Start Bit: 7, Length: 14 Byte Order: 0, Value Type: + Factor: 0.1 Offset: 0 Unit: N
}

void message_canrx_PROC_ETAS_VDC_Tq(uint8_t *RxData) {
   Throttle_Torque = ((RxData[2] & 15) << 6) | ((RxData[3] & 252) >> 2);   // Signal: Throttle_Torque Start Bit: 19, Length: 10 Byte Order: 0, Value Type: - Factor: 0.1 Offset: 0 Unit: Nm
   Torque_R = ((RxData[1] & 63) << 4) | ((RxData[2] & 240) >> 4);   // Signal: Torque_R Start Bit: 13, Length: 10 Byte Order: 0, Value Type: - Factor: 0.1 Offset: 0 Unit: Nm
   Torque_L = ((RxData[0] & 255) << 2) | ((RxData[1] & 192) >> 6);   // Signal: Torque_L Start Bit: 7, Length: 10 Byte Order: 0, Value Type: - Factor: 0.1 Offset: 0 Unit: Nm
}

void message_canrx_RAW_RECU_Data(uint8_t *RxData) {
   Susp_R_R_Bits = ((RxData[0] & 255) << 4) | ((RxData[1] & 240) >> 4);   // Signal: Susp_R_R_Bits Start Bit: 7, Length: 12 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bits
   Susp_R_L_Bits = ((RxData[1] & 15) << 8) | (RxData[2] << 0);   // Signal: Susp_R_L_Bits Start Bit: 11, Length: 12 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bits
}

void message_canrx_STAT_FECU_Keep_Alive(uint8_t *RxData) {
   FECU_LV = ((RxData[4] & 31) << 7) | ((RxData[5] & 254) >> 1);   // Signal: FECU_LV Start Bit: 36, Length: 12 Byte Order: 0, Value Type: + Factor: 0.008865 Offset: 0 Unit: V
   FECU_5V = ((RxData[2] & 1) << 11) | (RxData[3] << 3) | ((RxData[4] & 224) >> 5);   // Signal: FECU_5V Start Bit: 16, Length: 12 Byte Order: 0, Value Type: + Factor: 0.00241758 Offset: 0 Unit: V
   FECU_3V3 = ((RxData[1] & 31) << 7) | ((RxData[2] & 254) >> 1);   // Signal: FECU_3V3 Start Bit: 12, Length: 12 Byte Order: 0, Value Type: + Factor: 0.00080586 Offset: 0 Unit: V
   Shutdown_Setas = ((RxData[1] & 128) >> 7);   // Signal: Shutdown_Setas Start Bit: 15, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Shutdown_SC_BOTS = ((RxData[1] & 32) >> 5);   // Signal: Shutdown_SC_BOTS Start Bit: 13, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Shutdown_BSPD_Inertia = ((RxData[1] & 64) >> 6);   // Signal: Shutdown_BSPD_Inertia Start Bit: 14, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Front_Alive = ((RxData[0] & 255) >> 0);   // Signal: Front_Alive Start Bit: 7, Length: 8 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: -
}

void message_canrx_STAT_RECU_Keep_Alive(uint8_t *RxData) {
   RECU_LV = ((RxData[4] & 7) << 9) | (RxData[5] << 1) | ((RxData[6] & 128) >> 7);   // Signal: RECU_LV Start Bit: 34, Length: 12 Byte Order: 0, Value Type: + Factor: 0.008865 Offset: 0 Unit: V
   RECU_5V = ((RxData[3] & 127) << 5) | ((RxData[4] & 248) >> 3);   // Signal: RECU_5V Start Bit: 30, Length: 12 Byte Order: 0, Value Type: + Factor: 0.00241758 Offset: 0 Unit: V
   RECU_3V3 = ((RxData[1] & 7) << 9) | (RxData[2] << 1) | ((RxData[3] & 128) >> 7);   // Signal: RECU_3V3 Start Bit: 10, Length: 12 Byte Order: 0, Value Type: + Factor: 0.00080586 Offset: 0 Unit: V
   Shutdown_TSMS_TSMP = ((RxData[1] & 128) >> 7);   // Signal: Shutdown_TSMS_TSMP Start Bit: 15, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Shutdown_RightTS = ((RxData[1] & 64) >> 6);   // Signal: Shutdown_RightTS Start Bit: 14, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Shutdown_LeftTS = ((RxData[1] & 32) >> 5);   // Signal: Shutdown_LeftTS Start Bit: 13, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Shutdown_HVD = ((RxData[1] & 8) >> 3);   // Signal: Shutdown_HVD Start Bit: 11, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Shutdown_HVBox = ((RxData[1] & 16) >> 4);   // Signal: Shutdown_HVBox Start Bit: 12, Length: 1 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bit
   Rear_Alive = ((RxData[0] & 255) >> 0);   // Signal: Rear_Alive Start Bit: 7, Length: 8 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: -
}

void message_canrx_RAW_FECU_Data2(uint8_t *RxData) {
   BrakePressure2_Bits = ((RxData[6] & 255) << 4) | ((RxData[7] & 240) >> 4);   // Signal: BrakePressure2_Bits Start Bit: 55, Length: 12 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bits
   BrakePressure1_Bits = ((RxData[4] & 15) << 8) | (RxData[5] << 0);   // Signal: BrakePressure1_Bits Start Bit: 35, Length: 12 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bits
   Pitot_Bits = ((RxData[3] & 255) << 4) | ((RxData[4] & 240) >> 4);   // Signal: Pitot_Bits Start Bit: 31, Length: 12 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bits
   Susp_F_R_Bits = ((RxData[0] & 255) << 4) | ((RxData[1] & 240) >> 4);   // Signal: Susp_F_R_Bits Start Bit: 7, Length: 12 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bits
   Susp_F_L_Bits = ((RxData[1] & 15) << 8) | (RxData[2] << 0);   // Signal: Susp_F_L_Bits Start Bit: 11, Length: 12 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bits
}

void message_canrx_RAW_FECU_Data1(uint8_t *RxData) {
   SteeringSensor_Bits = ((RxData[4] & 15) << 8) | (RxData[5] << 0);   // Signal: SteeringSensor_Bits Start Bit: 35, Length: 12 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bits
   BrakePedal_Bits = ((RxData[3] & 255) << 4) | ((RxData[4] & 240) >> 4);   // Signal: BrakePedal_Bits Start Bit: 31, Length: 12 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bits
   APPS2_Bits = ((RxData[1] & 15) << 8) | (RxData[2] << 0);   // Signal: APPS2_Bits Start Bit: 11, Length: 12 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bits
   APPS1_Bits = ((RxData[0] & 255) << 4) | ((RxData[1] & 240) >> 4);   // Signal: APPS1_Bits Start Bit: 7, Length: 12 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: Bits
}

void message_canrx_STAT_ETAS_Sync(uint8_t *RxData) {
   ETAS_Sync_X = ((RxData[0] & 255) >> 0);   // Signal: ETAS_Sync_X Start Bit: 7, Length: 8 Byte Order: 0, Value Type: + Factor: 1 Offset: 0 Unit: -
}
//Error functions----------------------------------------------------------------------------------------
void error_handle(void) {
    if (error_count > 10) {
        error_count = 0;
        NVIC_SystemReset();
    }
}
