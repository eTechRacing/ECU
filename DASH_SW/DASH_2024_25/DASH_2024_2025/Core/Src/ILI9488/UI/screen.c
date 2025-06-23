#include "ILI9488/UI/screen.h"
#include <stdlib.h>
#include <stdio.h>
#include "DASH/etr_carstate.h"
#include "CAN/CAN_X_2025.h"

const char* status[] = {
		"ERROR", "OK"
};
const char* ecus_list[] = {
		"ETAS", "REAR", "FRONT", "DASH", "BMS"
};

const char* sensors_list[] = {
		"APPS1+2", "ELIPSE", "SUSP", "PITOT", "BRK PDL", "BRK PRS"
};

const char* shutdown_list[] = {
		"LVMS+STAS", "BSPD+INER", "SC+BOTS", "BMS", "IMD", "TSMS+TSMP", "LEFT TS", "RIGHT TS", "HV BOX", "HUD", "ACCU INT"
};

const char* dymaic_tests[] = {
		"SKIDPAD","AUTOX", "ENDURANCE", "ACCELERATION", "WORKSHOP"
};
uint8_t ecus_status[N_ECUS];
uint8_t sdown_status[N_SHUTDOWN];
uint8_t sensors_status[N_SENSORS];
void initStatus(){

	ecus_status[0] = 1/*ETAS_MSG_Counter*/;
	ecus_status[1] = Disconnection_Rear;
	ecus_status[2] = Disconnection_Front;
	ecus_status[3] = Disconnection_DashBoard;
	ecus_status[4] = Disconnection_BMS;

	sdown_status[0] = 0/*Shutdown_Setas*/;
	sdown_status[1] = 1/*Shutdown_BSPD_Inertia*/;
	sdown_status[2] = 0/*Shutdown_SC_BOTS*/;
	sdown_status[3] = 0/*Shutdown_BMS*/;
	sdown_status[4] = 1/*Shutdown_IMD*/;
	sdown_status[5] = 1/*Shutdown_TSMS_TSMP*/;
	sdown_status[6] = 1/*Shutdown_LeftTS*/;
	sdown_status[7] = 0/*Shutdown_RightTS*/;
	sdown_status[8] = 1/*Shutdown_HVBox*/;
	sdown_status[9] = 1/*Shutdown_HVD*/;
	sdown_status[10] = 1/*Shutdown_PackageIntck*/;
/*
	sensors_status[0] = Disconnection_APPS1 && Disconnection_APPS2;
	sensors_status[1] = Disconnection_Ellipse;
	sensors_status[2] = Disconnection_Susp_R_R && Disconnection_Susp_R_L && Disconnection_Susp_F_R && Disconnection_Susp_F_L;
	sensors_status[3] = Disconnection_Pitot;
	sensors_status[4] = Disconnection_BrakePedal;
	sensors_status[5] = Disconnection_BrakePressure1 && Disconnection_BrakePressure2;
*/
}


//UTILITY FUNCTIONS
const char* intToString (int value){
	static char buffer[12];
	sprintf(buffer, "%d", value);
	return buffer;
}
const char* floatToString (float value){
	static char buffer[30];
	int int_part = (int)value;
	int decimal_part = (int)((value-int_part)*100);
	if(decimal_part < 0) decimal_part = -decimal_part;

	sprintf(buffer,"%d.%02d", int_part, decimal_part);
	return buffer;
}

/////////////////////////
//----- CAR_STATE - 0
////////////////////////
void carState_0_SC0 (){
	// ----- SCREEN 1 -----
	ILI9488_FillScreen_DMA(0x0000);
	ILI9488_DrawBitmapRGB565(10,75,460,150,logo);
}
//REVISAR
void carState_0_SC1_ecus (int mode){
	//*************ECUS*************************//
	int y_pos = 70;
	uint16_t color = 0xFFFF;
	initStatus();
	if(mode==0){
	ILI9488_FillScreen_DMA(0x0000);

	ILI9488_DrawString(86,0,"CAR STATUS", Font32, 0xFFFF);
	ILI9488_Square(0, 63, 479, 65, 0xF800); //1H line
	ILI9488_Square(0, 212, 479, 215, 0xF800);//2H line
	ILI9488_Square(158, 63, 160, 212, 0xF800); //1V line
	ILI9488_Square(318, 63, 320, 212, 0xF800); //2V line
	ILI9488_DrawStringBold(52,43,"ECUS",Font16,0x0FFF);
	ILI9488_DrawStringBold(334,43,"SHUTDOWN",Font16,0x0FFF);
	ILI9488_DrawStringBold(192,192,"SENSORS",Font16,0x0FFF);

	}

	if(mode==1){

			//int n_ecus = sizeof(ecus_list)/sizeof(ecus_list[0]);

	for (int i=0;i<N_ECUS; i++){
			if(status[ecus_status[i]]==status[0]){
				color = 0xF800;
			}else{
				color = 0x07E0;
			}
			ILI9488_DrawStringBold(25, y_pos, ecus_list[i], Font16, 0xFFFF);
			ILI9488_FillCircle(12,y_pos+10, 10, color);
			y_pos+=23;
	}
	}
	//"LVMS+STAS", "BSPD+INER", "SC+BOTS", "BMS", "IMD", "TSMS+TSMP", "LEFT TS", "RIGHT TS", "HV BOX", "HVD", "ACCU INT"
	/*********** SHUTDOWN *****************/
	if (mode==2){
	//int n_sdown = sizeof(shutdown_list)/sizeof(shutdown_list[0]);
	y_pos = 222;
	int x_pos=12;
	for (int i=0;i<N_SHUTDOWN;i++){
		if (i==4||i==8) {
			x_pos += 165;
			y_pos=222;
		}
		if(status[sdown_status[i]]==status[0]){
			color = 0xF800;
		}else{
			color = 0x07E0;
		}
		ILI9488_DrawStringBold(x_pos+12, y_pos, shutdown_list[i], Font16, 0xFFFF);
		ILI9488_FillCircle(x_pos,y_pos+10, 10, color);
		y_pos+=23;
	}
	}
	//"APPS1+2", "ELIPSE", "SUSP", "PITOT", "BRK PDL", "BRK PRS"
	/**************** SENSORS *******************/
	if(mode==3){
	//int n_sens = sizeof(sensors_list)/sizeof(sensors_list[0]);
//	uint8_t sens_status[n_sens];
//		sens_status[0]=1/*Disconnection_APPS1 && Disconnection_APPS2*/;
//		sens_status[1]=0/*Disconnection_Ellipse*/;
//		sens_status[2]=1/*Disconnection_Susp_R_R && Disconnection_Susp_R_L && Disconnection_Susp_F_R && Disconnection_Susp_F_L */;
//		sens_status[3]=1/*Disconnection_Pitot*/;
//		sens_status[4]=1/*Disconnection_BrakePedal*/;
//		sens_status[5]=0/*Disconnection_BrakePressure1 && Disconnection_BrakePressure2*/;

	y_pos = 70;
	for (int i=0;i<N_SENSORS; i++){
			if(status[sensors_status[i]]==status[0]){
				color = 0xF800;
			}else{
				color = 0x07E0;
			}
			ILI9488_DrawStringBold(354, y_pos, sensors_list[i], Font16, 0xFFFF);
			ILI9488_FillCircle(342,y_pos+10, 10, color);

			y_pos+=23;
	}
	}
}



//MANDOS CENTRALES
void carState_0_SC2_refri (int status, DASH_refriSettings setup){
	//------... STRUCTURE------//
	/////////////////////////////
	////////|	1	|	2	|////
	//--------------------------
	//	1	|	x	|	x	|//// |= 70px // _ = 220px
	//--------------------------
	//	2	|	x	|	x	|////
	//--------------------------
	//	3	|		x		|////
	/////////////////////////////
	/*
	//Dynamix STATUS
	int L_fanStatus=2;
	int L_pumpStatus=1;
	int R_fanStatus=2;
	int R_pumpStatus=0;
	int accuRefri_status=2;
	int L_fan_0, L_fan_1;
	int L_pump_0, L_pump_1;
	int R_fan_0, R_fan_1;
	int R_pump_0, R_pump_1;
	int accuFan_0, accuFan_1;
	*/
	switch(setup.L_fanStatus){
					case 0:
						setup.L_fan_0 = 32;
						setup.L_fan_1 = 75;
						break;
					case 1:
						setup.L_fan_0 = 78;
						setup.L_fan_1 = 134;
						break;
					case 2:
						setup.L_fan_0 = 140;
						setup.L_fan_1 = 208;
						break;
	}
	switch(setup.L_pumpStatus){
					case 0:
						setup.L_pump_0 = 70;
						setup.L_pump_1 = 113;
						break;

					case 1:
						setup.L_pump_0 = 113;
						setup.L_pump_1 = 170;
						break;

	}
	switch(setup.R_fanStatus){
					case 0:
						setup.R_fan_0 = 272;
						setup.R_fan_1 = 315;
						break;
					case 1:
						setup.R_fan_0 = 318;
						setup.R_fan_1 = 374;
						break;
					case 2:
						setup.R_fan_0 = 380;
						setup.R_fan_1 = 448;
						break;
	}
	switch(setup.R_pumpStatus){
					case 0:
						setup.R_pump_0 = 310;
						setup.R_pump_1 = 353;
						break;

					case 1:
						setup.R_pump_0 = 353;
						setup.R_pump_1 = 410;
						break;

	}
	switch(setup.accuRefri_status){
					case 0:
						setup.accuFan_0 = 152;
						setup.accuFan_1 = 195;
						break;

					case 1:
						setup.accuFan_0 = 198;
						setup.accuFan_1 = 254;
						break;

					case 2:
						setup.accuFan_0 = 260;
						setup.accuFan_1 = 328;
						break;

	}


	if (status==0){
	ILI9488_FillScreen_DMA(0x0000);

	ILI9488_DrawStringBold(106,0,"L", Font32, 0xC7FF);
	ILI9488_Square(239,0,240,230,0xF800); //half V line
	ILI9488_DrawStringBold(346,0,"R", Font32, 0xC7FF);
	ILI9488_Square(0,49,480-1,50,0xF800); //top H line
	ILI9488_Square(0,229,479,230,0xF800);
	//square 1-1
	ILI9488_Square(10,60,230,130,0xFFFF);
	ILI9488_DrawStringBold(95,65,"FAN",Font16,0x0000);
	ILI9488_Square(30,100,210,120,0x0000);
	ILI9488_Square(setup.L_fan_0,102,setup.L_fan_1,118,0x07e0);
	ILI9488_DrawStringBold(40,97,"ON OFF FULL",Font16,0xFFFF);

	//square 1-2
	ILI9488_Square(10,150,230,220,0xFFFF);
	ILI9488_DrawStringBold(90,155,"PUMP",Font16,0x000);
	ILI9488_Square(68,190,172,210,0x0000);
	ILI9488_Square(setup.L_pump_0,192,setup.L_pump_1,208,0x07e0);
	ILI9488_DrawStringBold(78,187,"ON OFF",Font16,0xFFFF);

	//square 2-1
	ILI9488_Square(250,60,470,130,0xFFFF);
	ILI9488_DrawStringBold(335,65,"FAN",Font16,0x0000);
	ILI9488_Square(270,100,450,120,0x0000);
	ILI9488_Square(setup.R_fan_0,102,setup.R_fan_1,118,0x07e0);
	ILI9488_DrawStringBold(280,97,"ON OFF FULL",Font16,0xFFFF);
	//square 2-2
	ILI9488_Square(250,150,470,220,0xFFFF);
	ILI9488_DrawStringBold(330,155,"PUMP",Font16,0x000);
	ILI9488_Square(308,190,412,210,0x0000);
	ILI9488_Square(setup.R_pump_0,192,setup.R_pump_1,208,0x07e0);
	ILI9488_DrawStringBold(318,187,"ON OFF",Font16,0xFFFF);
	//square 3
	ILI9488_Square(10,240,464,312,0xFFFF);
	ILI9488_DrawStringBold(163,240,"ACCUMULATOR",Font16,0x0000);
	ILI9488_DrawStringBold(219,260,"FAN",Font16,0x0000);
	ILI9488_Square(150,287,330,307,0x0000);
	ILI9488_Square(setup.accuFan_0,289,setup.accuFan_1,305,0x07e0);
	ILI9488_DrawStringBold(160,285,"ON OFF FULL",Font16,0xFFFF);
	}else if(status==1){
		ILI9488_Square(10,60,230,130,0xCCFF);
		ILI9488_DrawStringBold(95,65,"FAN",Font16,0x0000);
		ILI9488_Square(30,100,210,120,0x0000);
		ILI9488_Square(setup.L_fan_0,102,setup.L_fan_1,118,0x07e0);
		ILI9488_DrawStringBold(40,97,"ON OFF FULL",Font16,0xFFFF);
	}else if(status==2){
		ILI9488_Square(10,150,230,220,0xCCFF);
		ILI9488_DrawStringBold(90,155,"PUMP",Font16,0x000);
		ILI9488_Square(68,190,172,210,0x0000);
		ILI9488_Square(setup.L_pump_0,192,setup.L_pump_1,208,0x07e0);
		ILI9488_DrawStringBold(78,187,"ON OFF",Font16,0xFFFF);
	}else if(status==3){
		ILI9488_Square(10,240,464,312,0xCCFF);
		ILI9488_DrawStringBold(163,240,"ACCUMULATOR",Font16,0x0000);
		ILI9488_DrawStringBold(219,260,"FAN",Font16,0x0000);
		ILI9488_Square(150,287,330,307,0x0000);
		ILI9488_Square(setup.accuFan_0,289,setup.accuFan_1,305,0x07e0);
		ILI9488_DrawStringBold(160,285,"ON OFF FULL",Font16,0xFFFF);
	}else if(status==4){
		ILI9488_Square(250,60,470,130,0xCCFF);
		ILI9488_DrawStringBold(335,65,"FAN",Font16,0x0000);
		ILI9488_Square(270,100,450,120,0x0000);
		ILI9488_Square(setup.R_fan_0,102,setup.R_fan_1,118,0x07e0);
		ILI9488_DrawStringBold(280,97,"ON OFF FULL",Font16,0xFFFF);
	}else if(status==5){
		ILI9488_Square(250,150,470,220,0xCCFF);
		ILI9488_DrawStringBold(330,155,"PUMP",Font16,0x000);
		ILI9488_Square(308,190,412,210,0x0000);
		ILI9488_Square(setup.R_pump_0,192,setup.R_pump_1,208,0x07e0);
		ILI9488_DrawStringBold(318,187,"ON OFF",Font16,0xFFFF);
	}
}

/////////////////////////
//----- CAR_STATE - 3
////////////////////////
//CENTRAL BUTTON TO START
void carState_3_SC0 (){
	// ----- SCREEN 2 -----
	ILI9488_FillScreen_DMA(0x0000);
	ILI9488_DrawBitmapRGB565(44,50,392,220,precharge_0);

}


/////////////////////////
//----- CAR_STATE - 6
////////////////////////
void carState_6 (){
	ILI9488_FillScreen_DMA(0x0000);
		ILI9488_DrawBitmapRGB565(102.5,30,275,110,precharge_1);
		int i = 0;
		int x = 46;
		int y = 197;
		while (i < 8){
			ILI9488_Square(x, y, x+45, y+30, 0x07E0);
			x+=49;
			i++;
			HAL_Delay(800);
		}
}

/////////////////////////
//----- CAR_STATE - 9
////////////////////////
void carState_9 (){
	ILI9488_DrawString(137, 100, "PRECHARGE FINISHED", Font24, 0xFFFF);
}

/////////////////////////
//----- CAR_STATE - 12
////////////////////////
void carState_12_DYNAMIC (int screen, int column){
	ILI9488_FillScreen_DMA(0x0000);
	/*
	if(column == 0){
		//FOR THE RACE TYPE
	}else if(column == 1){
		//FOR THE DRIVER SELECT
	}else if(column == 2){
		//FOR THE FINAL SCREEN
	}
	*/
	ILI9488_DrawString(137,100,dymaic_tests[screen],Font24,0xFFFF);
}
void carState_12_DRIVER (int screen, int column){
	ILI9488_FillScreen_DMA(0x0000);
	/*
	if(column == 0){
		//FOR THE RACE TYPE
	}else if(column == 1){
		//FOR THE DRIVER SELECT
	}else if(column == 2){
		//FOR THE FINAL SCREEN
	}
	*/
	ILI9488_DrawString(137,100,"DRIVER",Font24,0xFFFF);
	ILI9488_DrawString(250,100,intToString(column),Font24,0xFFFF);
}
/////////////////////////
//----- CAR_STATE - 14
////////////////////////
void carState_14 (){
	ILI9488_FillScreen_DMA(0x0000);
	ILI9488_DrawString(137, 100, "INVERTERS GETTING READY", Font24, 0xFFFF);
}

/////////////////////////
//----- CAR_STATE - 15
////////////////////////
void carState_15 (int n_race){
	ILI9488_FillScreen_DMA(0xFFFF);
	if (n_race == 0){
		//ACCELERATION
		ILI9488_Square(0,0,152,80,0x001F);
		ILI9488_Square(328,0,479,80,0xF800);
	}else if(n_race == 1){
		//SKIDPAD

	}else if(n_race == 2){
		//AUTOCROSS
	}else if(n_race == 3){
		//ENDURANCE
	}

}

/////////////////////////
//----- CAR_STATE - 21
////////////////////////
void carState_21 (){
	ILI9488_FillScreen_DMA(0x0000);
	ILI9488_DrawString(137, 100, "ERROR SOMETHING WENT RONG", Font24, 0xFFFF);
}

