#include "ILI9488/UI/screen.h"
#include <stdlib.h>
#include <stdio.h>
#include "CAN/CAN_X_2025.h"
#include "CAN/CAN.h"
#include "DASH/etr_carstate.h"
#include <string.h>
const char* status[] = {
		"ERROR", "OK"
};
const char* ecus_list[] = {
		"ETAS", "REAR", "FRONT", "DASH", "BMS"
};

const char* sensors_list[] = {
		"APPS1+2", "ELIPSE", "SUSPE", "PITOT", "BRK PDL", "BRK PRS"
};

const char* shutdown_list[] = {
		"LVMS+STAS", "BSPD+INER", "SC+BOTS", "BMS", "IMD", "TSMS+TSMP", "LEFT TS", "RIGHT TS", "HV BOX", "HUD", "ACCU INT"
};

const char* dymaic_tests[] = {
		"SKIDPAD","AUTOX", "ENDURANCE", "ACCELERATION", "WORKSHOP"
};
const char* driver_name[] = {
		"DAVID","JOSE","JORDI","DRIVER 4"
};
uint8_t ecus_status[N_ECUS];
uint8_t sdown_status[N_SHUTDOWN];
uint8_t sensors_status[N_SENSORS];
void initStatus(){

	ecus_status[0] = Disconnection_ETAS;
	ecus_status[1] = Disconnection_Rear;
	ecus_status[2] = Disconnection_Front;
	ecus_status[3] = Disconnection_DashBoard;
	ecus_status[4] = Disconnection_BMS;

	sdown_status[0] = Shutdown_Setas;
	sdown_status[1] = Shutdown_BSPD_Inertia;
	sdown_status[2] = Shutdown_SC_BOTS;
	sdown_status[3] = Shutdown_BMS;
	sdown_status[4] = Shutdown_IMD;
	sdown_status[5] = Shutdown_TSMS_TSMP;
	sdown_status[6] = Shutdown_LeftTS;
	sdown_status[7] = Shutdown_RightTS;
	sdown_status[8] = Shutdown_HVBox;
	sdown_status[9] = Shutdown_HVD;
	sdown_status[10] = Shutdown_PackageIntck;

	sensors_status[0] = Disconnection_APPS1 || Disconnection_APPS2;
	sensors_status[1] = Disconnection_Ellipse;
	sensors_status[2] = Disconnection_Susp_R_R || Disconnection_Susp_R_L || Disconnection_Susp_F_R || Disconnection_Susp_F_L;
	sensors_status[3] = Disconnection_Pitot;
	sensors_status[4] = Disconnection_BrakePedal;
	sensors_status[5] = Disconnection_BrakePressure1 && Disconnection_BrakePressure2;

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
	ILI9488_DrawString(170, 219, "E", Font32, 0x07E0);
	ILI9488_DrawString(170, 219," TECH", Font32,0xFFFF);
	//ILI9488_DrawBitmapRGB565(10,75,460,150,logo);
}
//REVISAR
void carState_0_SC1_ecus (int mode){
	//*************ECUS*************************//

	int y_pos = 70;
	uint16_t color = 0xFFFF;
	initStatus();
	if(printStatus>3&& printStatus%2==0){
		for (int i=0;i<N_ECUS; i++){
			if(status[ecus_status[i]]==status[0]){
				color = 0x07E0;
			}else{
				color = 0xF800;
			}
			ILI9488_FillCircle(12,y_pos+10, 10, color);
			y_pos+=23;
		}
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
			ILI9488_FillCircle(x_pos,y_pos+10, 10, color);
			y_pos+=23;
		}
		}else if((printStatus>3)&(printStatus%2!=0)){
		y_pos = 70;
		for (int i=0;i<N_SENSORS; i++){
				if(status[sensors_status[i]]==status[0]){
					color = 0xF800;
				}else{
					color = 0x07E0;
				}
				ILI9488_FillCircle(342,y_pos+10, 10, color);
				y_pos+=23;
		}
		}


	switch (mode){
	case(0):
		ILI9488_FillScreen_DMA(0x0000);
		ILI9488_DrawString(86,0,"CAR STATUS", Font32, 0xFFFF);
		ILI9488_Square(0, 63, 479, 65, 0xF800); //1H line
		ILI9488_Square(0, 212, 479, 215, 0xF800);//2H line
		ILI9488_Square(158, 63, 160, 212, 0xF800); //1V line
		ILI9488_Square(318, 63, 320, 212, 0xF800); //2V line
		ILI9488_DrawStringBold(52,43,"ECUS",Font16,0x0FFF);
		ILI9488_DrawStringBold(334,43,"SHUTDOWN",Font16,0x0FFF);
		ILI9488_DrawStringBold(192,192,"SENSORS",Font16,0x0FFF);
	case(1):
		for (int i=0;i<N_ECUS; i++){
			if(status[ecus_status[i]]==status[0]){
				color = 0x07E0;
			}else{
				color = 0xF800;
			}
			ILI9488_DrawStringBold(25, y_pos, ecus_list[i], Font16, 0xFFFF);
			ILI9488_FillCircle(12,y_pos+10, 10, color);
			y_pos+=23;
		}
		break;
	case(2):
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
		break;
	case(3):
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
		break;

	}


//	if(mode==0){
//	ILI9488_FillScreen_DMA(0x0000);
//
//	ILI9488_DrawString(86,0,"CAR STATUS", Font32, 0xFFFF);
//	ILI9488_Square(0, 63, 479, 65, 0xF800); //1H line
//	ILI9488_Square(0, 212, 479, 215, 0xF800);//2H line
//	ILI9488_Square(158, 63, 160, 212, 0xF800); //1V line
//	ILI9488_Square(318, 63, 320, 212, 0xF800); //2V line
//	ILI9488_DrawStringBold(52,43,"ECUS",Font16,0x0FFF);
//	ILI9488_DrawStringBold(334,43,"SHUTDOWN",Font16,0x0FFF);
//	ILI9488_DrawStringBold(192,192,"SENSORS",Font16,0x0FFF);
//
//	}

//	if(mode==1){
//	for (int i=0;i<N_ECUS; i++){
//			if(status[ecus_status[i]]==status[0]){
//				color = 0xF800;
//			}else{
//				color = 0x07E0;
//			}
//			ILI9488_DrawStringBold(25, y_pos, ecus_list[i], Font16, 0xFFFF);
//			ILI9488_FillCircle(12,y_pos+10, 10, color);
//			y_pos+=23;
//	}
//	}
	//"LVMS+STAS", "BSPD+INER", "SC+BOTS", "BMS", "IMD", "TSMS+TSMP", "LEFT TS", "RIGHT TS", "HV BOX", "HVD", "ACCU INT"
	/*********** SHUTDOWN *****************/
//	if (mode==2){
//	y_pos = 222;
//	int x_pos=12;
//	for (int i=0;i<N_SHUTDOWN;i++){
//		if (i==4||i==8) {
//			x_pos += 165;
//			y_pos=222;
//		}
//		if(status[sdown_status[i]]==status[0]){
//			color = 0xF800;
//		}else{
//			color = 0x07E0;
//		}
//		ILI9488_DrawStringBold(x_pos+12, y_pos, shutdown_list[i], Font16, 0xFFFF);
//		ILI9488_FillCircle(x_pos,y_pos+10, 10, color);
//		y_pos+=23;
//	}
//	}
	//"APPS1+2", "ELIPSE", "SUSP", "PITOT", "BRK PDL", "BRK PRS"
	/**************** SENSORS *******************/
//	if(mode==3){
//	y_pos = 70;
//	for (int i=0;i<N_SENSORS; i++){
//			if(status[sensors_status[i]]==status[0]){
//				color = 0xF800;
//			}else{
//				color = 0x07E0;
//			}
//			ILI9488_DrawStringBold(354, y_pos, sensors_list[i], Font16, 0xFFFF);
//			ILI9488_FillCircle(342,y_pos+10, 10, color);
//
//			y_pos+=23;
//	}
//	}
//}
}


//MANDOS CENTRALES
void carState_0_SC2_refri (){
	// ----- SCREEN 3 -----
//	ILI9488_FillScreen_DMA(0x0000);
//
//	ILI9488_DrawStringBold(106,0,"L", Font32, 0xC7FF);
//	ILI9488_Square(239,0,240,230,0xF800); //half V line
//	ILI9488_DrawStringBold(346,0,"R", Font32, 0xC7FF);
//	ILI9488_Square(0,49,480-1,50,0xF800); //top H line
//	ILI9488_Square(0,229,479,230,0xF800);
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
	//Dynamix STATUS
	int L_fanStatus=Fans_L;
	int L_pumpStatus=!Pump_L;
	int R_fanStatus=Fans_R;
	int R_pumpStatus=!Pump_R;
	int accuRefri_status=Refri_ACCU;
	int L_fan_0, L_fan_1;
	int L_pump_0, L_pump_1;
	int R_fan_0, R_fan_1;
	int R_pump_0, R_pump_1;
	int accuFan_0, accuFan_1;

	switch(L_fanStatus){
					case 0:
						L_fan_0 = 78;
						L_fan_1 = 134;
						break;
					case 1:
						L_fan_0 = 32;
						L_fan_1 = 75;
						break;
					case 2:
						L_fan_0 = 140;
						L_fan_1 = 208;
						break;
	}
	switch(L_pumpStatus){
					case 0:
						L_pump_0 = 70;
						L_pump_1 = 113;
						break;

					case 1:
						L_pump_0 = 113;
						L_pump_1 = 170;
						break;

	}
	switch(R_fanStatus){
					case 0:
						R_fan_0 = 318;
						R_fan_1 = 374;
						break;
					case 1:
						R_fan_0 = 272;
						R_fan_1 = 315;
						break;
					case 2:
						R_fan_0 = 380;
						R_fan_1 = 448;
						break;
	}
	switch(R_pumpStatus){
					case 0:
						R_pump_0 = 310;
						R_pump_1 = 353;
						break;

					case 1:
						R_pump_0 = 353;
						R_pump_1 = 410;
						break;

	}
	switch(accuRefri_status){
					case 0:
						accuFan_0 = 198;
						accuFan_1 = 254;
						break;

					case 1:
						accuFan_0 = 152;
						accuFan_1 = 195;
						break;

					case 2:
						accuFan_0 = 260;
						accuFan_1 = 328;
						break;

}

	if(printStatus==0){
		if(Screen.RefriSettings == 0){
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
			ILI9488_Square(L_fan_0,102,L_fan_1,118,0x07e0);
			ILI9488_DrawStringBold(40,97,"ON OFF FULL",Font16,0xFFFF);

			//square 1-2
			ILI9488_Square(10,150,230,220,0xFFFF);
			ILI9488_DrawStringBold(90,155,"PUMP",Font16,0x000);
			ILI9488_Square(68,190,172,210,0x0000);
			ILI9488_Square(L_pump_0,192,L_pump_1,208,0x07e0);
			ILI9488_DrawStringBold(78,187,"ON OFF",Font16,0xFFFF);

			//square 2-1
			ILI9488_Square(250,60,470,130,0xFFFF);
			ILI9488_DrawStringBold(335,65,"FAN",Font16,0x0000);
			ILI9488_Square(270,100,450,120,0x0000);
			ILI9488_Square(R_fan_0,102,R_fan_1,118,0x07e0);
			ILI9488_DrawStringBold(280,97,"ON OFF FULL",Font16,0xFFFF);
			//square 2-2
			ILI9488_Square(250,150,470,220,0xFFFF);
			ILI9488_DrawStringBold(330,155,"PUMP",Font16,0x000);
			ILI9488_Square(308,190,412,210,0x0000);
			ILI9488_Square(R_pump_0,192,R_pump_1,208,0x07e0);
			ILI9488_DrawStringBold(318,187,"ON OFF",Font16,0xFFFF);
			//square 3
			ILI9488_Square(10,240,464,312,0xFFFF);
			ILI9488_DrawStringBold(163,240,"ACCUMULATOR",Font16,0x0000);
			ILI9488_DrawStringBold(219,260,"FAN",Font16,0x0000);
			ILI9488_Square(150,287,330,307,0x0000);
			ILI9488_Square(accuFan_0,289,accuFan_1,305,0x07e0);
			ILI9488_DrawStringBold(160,285,"ON OFF FULL",Font16,0xFFFF);

		}

	}else if (printStatus >=1){
		if(Screen.RefriSettings == 1){
			switch(Screen.refriSetup.system){
				case(0):
					ILI9488_Square(30,100,210,120,0x0000);
					ILI9488_Square(L_fan_0,102,L_fan_1,118,0x07e0);
					ILI9488_DrawStringBold(40,97,"ON OFF FULL",Font16,0xFFFF);
				break;
				case(1):
					ILI9488_Square(68,190,172,210,0x0000);
					ILI9488_Square(L_pump_0,192,L_pump_1,208,0x07e0);
					ILI9488_DrawStringBold(78,187,"ON OFF",Font16,0xFFFF);
				break;
				case(2):
					ILI9488_Square(270,100,450,120,0x0000);
					ILI9488_Square(R_fan_0,102,R_fan_1,118,0x07e0);
					ILI9488_DrawStringBold(280,97,"ON OFF FULL",Font16,0xFFFF);
				break;
				case(3):
					ILI9488_Square(308,190,412,210,0x0000);
					ILI9488_Square(R_pump_0,192,R_pump_1,208,0x07e0);
					ILI9488_DrawStringBold(318,187,"ON OFF",Font16,0xFFFF);
				break;
				case(4):

					ILI9488_DrawStringBold(219,260,"FAN",Font16,0x0000);
					ILI9488_Square(150,287,330,307,0x0000);
					ILI9488_Square(accuFan_0,289,accuFan_1,305,0x07e0);
					ILI9488_DrawStringBold(160,285,"ON OFF FULL",Font16,0xFFFF);
				break;
			}
		}
	}
}

/////////////////////////
//----- CAR_STATE - 3
////////////////////////
//CENTRAL BUTTON TO START
void carState_3_SC0 (){
	// ----- SCREEN 2 -----
	ILI9488_FillScreen_DMA(0x0000);
	ILI9488_DrawStringBold(170, 50, "PRESS", Font32, 0x07E0);
	ILI9488_DrawStringBold(0,105, " CENTRAL BUTTON", Font32, 0xFFFF);
	ILI9488_DrawStringBold(128,160,"TO START", Font32, 0xFFFF);
	ILI9488_DrawStringBold(114,215 , "PRECHARGE", Font32, 0x07E0);

}


/////////////////////////
//----- CAR_STATE - 6
////////////////////////
void carState_6 (){
	int x = 40;
	int y = 197;
	if(printStatus==0){
		ILI9488_FillScreen_DMA(0x0000);
		ILI9488_DrawStringBold(114,40,"PRECHARGE", Font32, 0xFFFF);
		ILI9488_DrawStringBold(156,90,"STATUS", Font32, 0xFFFF);

	}else{
//		while (i < 8){

			ILI9488_Square(x, y, ((int)Precharge_Percentage)*4, y+30, 0x07E0);
//			x+=49;
//			i++;
//			HAL_Delay(800);
	}
}

/////////////////////////
//----- CAR_STATE - 9
////////////////////////
void carState_9 (){
	ILI9488_FillScreen_DMA(0x0000);
	ILI9488_DrawString(114, 108, "PRECHARGE", Font32, 0xFFFF);
	ILI9488_DrawString(139, 155, "FINISHED", Font32, 0x07E0);
}

/////////////////////////
//----- CAR_STATE - 12
////////////////////////
void carState_12_DRIVER (int n_driver){
//	if(printStatus==0){
//		ILI9488_FillScreen_DMA(0x0000);
//		ILI9488_DrawString(137,100,"DRIVER",Font24,0xFFFF);
//	}else if(printStatus==1){
//		ILI9488_Square(250, 100, 267, 124, 0x0000);
//	}else{
//		ILI9488_DrawString(250,100,driver_name[n_driver],Font32,0xFFFF);
//		Driver=n_driver+1;
//	}

	ILI9488_FillScreen_DMA(0x0000);
	ILI9488_Square(0,0,419,1,0xF800);//H1
	ILI9488_Square(0,318,419,320,0xF800);//H4
	ILI9488_Square(0,0,1,319,0xF800);//V1
	ILI9488_Square(418,0,419,319,0xF800);//V4

	ILI9488_Square(0,118,419,119,0xF800);//H2
	ILI9488_Square(0,218,419,219,0xF800);//H3
	ILI9488_Square(159,118,160,319,0xF800);//V2
	ILI9488_Square(359,119,360,319,0xF800);//V3

	ILI9488_DrawStringBold(30,10,"DRIVER SETTINGS",Font32,0xFFFF);

	ILI9488_DrawString(24,193,"DRIVER 1",Font16,0xFFFF);
	ILI9488_DrawString(184,193,"DRIVER 2",Font16,0xFFFF);
	ILI9488_DrawString(344,193,"DRIVER 3",Font16,0xFFFF);
	ILI9488_DrawString(24,294,"DRIVER 4",Font16,0xFFFF);
	ILI9488_DrawString(184,294,"DRIVER 5",Font16,0xFFFF);
	ILI9488_DrawString(344,294,"DRIVER 6",Font16,0xFFFF);

}
/////////////////////////
//----- CAR_STATE - 14
////////////////////////
void carState_14 (){
	ILI9488_FillScreen_DMA(0x0000);

	ILI9488_DrawString(114,87,"INVERTERS", Font32, 0x07E0);
	ILI9488_DrawString(142,139,"GETTING", Font32, 0xFFFF);
	ILI9488_DrawString(170,191,"READY", Font32, 0x07E0);
}

/////////////////////////
//----- CAR_STATE - 15
////////////////////////
void carState_15 (int n_race){

	if(printStatus==0){
		ILI9488_FillScreen_DMA(0x0000);
		ILI9488_Square(239,0,240,319,0xF800);
		ILI9488_Square(0,159,479,160,0xF800);
		ILI9488_Square(0,0,1,319,0xF800);
		ILI9488_Square(0,0,479,1,0xF800);
		ILI9488_Square(478,0,479,319,0xF800);
		ILI9488_Square(0,319,479,319,0xF800);

		ILI9488_DrawBitmapMono(57, 30, skidpad, 125, 125, 0xFC00);
		ILI9488_DrawBitmapMono(297, 20, autox, 125, 125, 0x07E0);
		ILI9488_DrawBitmapMono(57, 190, endurance, 125, 125, 0x001F);
		ILI9488_DrawBitmapMono(297,190, acceleration, 125, 125, 0xF800);

		ILI9488_DrawStringBold(71,5,"WORKSHOP",Font16,0xFFFF);
		ILI9488_DrawStringBold(325, 5, "AUTOX", Font16, 0xFFFF);
		ILI9488_DrawStringBold(57, 165, "ENDURANCE", Font16, 0xFFFF);
		ILI9488_DrawStringBold(276, 165, "ACCELERATION", Font16, 0xFFFF);
		ILI9488_DrawStringBold(86,299,"DO NOTHING TO WORKSHOP",Font16,0x008F);
	}else if (printStatus>0){
		if(Screen.RaceSettings==1){
		switch(n_race){
			case(0):
//				ILI9488_DrawStringBold(325, 5, "AUTOX", Font16, 0xFFFF); //Avoid false visual active n_race in Screen
				ILI9488_DrawStringBold(71,5,"SKIPAD",Font16,0xF800);
				ILI9488_DrawStringBold(71,5,"SKIPAD",Font16,0xFFFF);
				RacingMode=2;

			break;
			case(1):
//				ILI9488_DrawStringBold(71,5,"SKIDPAD",Font16,0xFFFF);
//				ILI9488_DrawStringBold(57, 165, "ENDURANCE", Font16, 0xFFFF);
				ILI9488_DrawStringBold(325, 5, "AUTOX", Font16, 0xF800);
				ILI9488_DrawStringBold(325, 5, "AUTOX", Font16, 0xFFFF);
				RacingMode=4;
			break;
			case(2):
//				ILI9488_DrawStringBold(325, 5, "AUTOX", Font16, 0xFFFF);
//				ILI9488_DrawStringBold(276, 165, "ACCELERATION", Font16, 0xFFFF);
				ILI9488_DrawStringBold(57, 165, "ENDURANCE", Font16, 0xF800);
				ILI9488_DrawStringBold(57, 165, "ENDURANCE", Font16, 0xFFFF);
				RacingMode=5;
			break;
			case(3):
//				ILI9488_DrawStringBold(57, 165, "ENDURANCE", Font16, 0xFFFF);
				ILI9488_DrawStringBold(276, 165, "ACCELERATION", Font16, 0xF800);
				ILI9488_DrawStringBold(276, 165, "ACCELERATION", Font16, 0xFFFF);
				RacingMode=3;
			break;
		}
		}
		if(printStatus%2==0){ILI9488_DrawStringBold(86,299,"DO NOTHING TO WORKSHOP",Font16,0x008F);}else{ILI9488_DrawStringBold(86,299,"DO NOTHING TO WORKSHOP",Font16,0xFFFF);};

	}
}

void carState4_SC3 (){
	ILI9488_FillScreen_DMA(0x0000);
	ILI9488_DrawStringBold(170, 50, "PRESS", Font32, 0x07E0);
	ILI9488_DrawStringBold(0,105, " CENTRAL BUTTON", Font32, 0xFFFF);
	ILI9488_DrawStringBold(198,160,"AND", Font32, 0xFFFF);
	ILI9488_DrawStringBold(170,215 , "BRAKE", Font32, 0x07E0);
}
/////////////////////////
//----- CAR_STATE - 21
////////////////////////
char b_SteeringSensor_Value[20];

void carState_21 (){
	ILI9488_FillScreen_DMA(0x0000);
	ILI9488_DrawString(170, 139, "ERROR", Font32, 0xF800);

}
int prev_accel=0;
int accel_ad=0;
void screen_workshop(){
uint16_t c_steering;
if(printStatus==0){

	ILI9488_FillScreen_DMA(0x0000);

	ILI9488_Square(0, 0, 1, 319, 0xFFFF);//V1
	ILI9488_Square(0, 318,479, 319, 0xFFFF);//H3
	ILI9488_Square(0,0,479,1,0xFFFF);//H1
	ILI9488_Square(478,0,479,319,0xFFFF);//V6

	ILI9488_Square(83, 0, 84, 319, 0xFFFF);//V2
	ILI9488_Square(167, 0, 168, 319, 0xFFFF);//V3
	ILI9488_Square(311, 0, 312, 319, 0xFFFF);//V4
	ILI9488_Square(395, 0, 396, 319, 0xFFFF);//V5
	ILI9488_Square(0, 159, 84, 161, 0xFFFF);//H2.1
	ILI9488_Square(395, 159, 479, 161, 0xFFFF);//VH2.2

	ILI9488_Square(100,30,151,280, 0xf800);//BPPS
	ILI9488_Square(328,30,379,280, 0x0ff0);//APPS
	  	sprintf( b_SteeringSensor_Value,"%d#", SteeringSensor_Value);
	ILI9488_DrawString(184, 238, b_SteeringSensor_Value, Font32, 0xFFFF);//STEERING ANGLE
	ILI9488_DrawString(198,30,"99%",Font32,0xFFFF); //SOC
	ILI9488_DrawString(13, 58, "27",Font32,0xFFFF);// Torque L
	ILI9488_DrawString(410, 58, "27",Font32,0xFFFF);// Torque L
	ILI9488_DrawString(10,219,"5",Font32, 0xFFFF);
	ILI9488_DrawString(45,219,"5",Font32, 0xFFFF);
	ILI9488_DrawBitmapMono(210,130, logo2, 60, 60, 0xFFFF);
}else{
		accel_ad=APPS1_Value;
		if(accel_ad != prev_accel){
			ILI9488_Square(100,30,151,280, 0xf800);
//  			ILI9488_Square(102,(accel_ad*10)+32,149,275, 0x0000);
			if(accel_ad<=24){
			ILI9488_Square(102,275-(accel_ad*10),149,275, 0x0000);
			}
		}
		accel_ad=prev_accel;
		//Steering Sensor-------------------------
		ILI9488_Square(184, 238, 296, 280, 0x0000);
		sprintf( b_SteeringSensor_Value,"%d#", SteeringSensor_Value);
		if(SteeringSensor_Value<0){c_steering=0xF800;}else {c_steering=0xFFFF;};
		ILI9488_DrawString(184, 238, b_SteeringSensor_Value, Font32, c_steering);//STEERING ANGLE
		//----------------------------------------
}
}
char b_velocity[20];
int velocity=0.0;
char b_sensorics[20];
void screen_skipad(){
	if(printStatus==0){
	 ILI9488_FillScreen_DMA(0x0000);
	  ILI9488_Square(0,0,479,1,0xFFFF);//H1
	  ILI9488_Square(0, 0, 1, 319, 0xFFFF);//V1
	  ILI9488_Square(0, 318,479, 319, 0xFFFF);//H3
	  ILI9488_Square(478,0,479,319,0xFFFF);//V3

	  ILI9488_Square(239,99,240,219,0xFFFF);//V2
	  ILI9488_Square(0,99,479,100,0xFFFF);//H2
	  ILI9488_Square(0,217,479,219,0xFFFF);//H3

	  //ALERTA BIEN
	  ILI9488_Square(2,219,477,317,0x07E0);
	  ILI9488_DrawString(52,246,"ETECH RACING",Font32,0xFFFF);
	  ILI9488_DrawBitmapMono(400, 240, logo2, 60,60, 0xFFFF);
	}else{
	  //Cuadro alerta verde
//	  ILI9488_Square(2,219,477,317,0x07E0);
//	  ILI9488_DrawString(52,246,"ETECH RACING",Font32,0xFFFF);
//	  ILI9488_DrawBitmapMono(400, 240, logo2, 60,60, 0xFFFF);
	  //Cuadro alerta ROJO
	//  ILI9488_Square(2,219,477,317,0xF800);
	//  ILI9488_DrawString(114,220,"HIGH TEMP",Font32,0xD59F);
	//  ILI9488_DrawString(86,270,"LOW VOLTAGE",Font32,0xD59F); //LILA BRILLANTE

	  //STEERING
	  sprintf( b_SteeringSensor_Value,"%d#", SteeringSensor_Value);

	  if(SteeringSensor_Value>99){
	  ILI9488_Square(194,26,306,68,0x0000);
	  ILI9488_DrawStringBold(194,26,b_SteeringSensor_Value,Font32,0xFFFF); //>99
	  }else if(SteeringSensor_Value>9){
	  ILI9488_Square(208,26,290,68,0x0000);
	  ILI9488_DrawStringBold(208,26,b_SteeringSensor_Value,Font32,0xFFFF); //>9
	  }else if(SteeringSensor_Value>=0){
	  ILI9488_Square(222,26,278,68,0x0000);
	  ILI9488_DrawStringBold(222,26,b_SteeringSensor_Value,Font32,0xFFFF);//>=0
	  }

	  //VELOCITY
	  velocity=el_VEL*3.6;
	  sprintf(b_velocity,"%d",velocity);

	  ILI9488_DrawStringBold(179,195,"km/h",Font16,0x5F00);
	  ILI9488_DrawStringBold(74,136,b_velocity,Font32,0xFFFF);

	  //SENSORICS MODE

	  sprintf(b_sensorics,"%d",Sensorics_Mode);
	  ILI9488_DrawStringBold(446,195,"SM",Font16,0x5F00);
	  ILI9488_DrawStringBold(346,134,b_sensorics,Font32,0xFFFF);
	}
}

void screen_acceleration(){
	ILI9488_FillScreen_DMA(0x0000);

	ILI9488_Square(0,0,479,1,0xFFFF);//H1
	ILI9488_Square(0, 0, 1, 319, 0xFFFF);//V1
	ILI9488_Square(0, 318,479, 319, 0xFFFF);//H3
	ILI9488_Square(478,0,479,319,0xFFFF);//V3

	//Traction Control
	ILI9488_Square(0,0,479,99,0xF800);
	ILI9488_DrawStringBold(114,29,"TC ON/OFF",Font32,0xFFFF);
	//Square 3
	ILI9488_Square(2,219,477,317,0x07E0);
	ILI9488_DrawString(52,246,"ETECH RACING",Font32,0xFFFF);
	ILI9488_DrawBitmapMono(400, 240, logo2, 60,60, 0xFFFF);

	//Velocity
	velocity=el_VEL*3.6;
	sprintf(b_velocity,"%d",velocity);
	ILI9488_DrawStringBold(179,195,"km/h",Font16,0x5F00);
	if(el_VEL>9){
		ILI9488_DrawStringBold(92,136,b_velocity,Font32,0xFFFF);
	}else if(el_VEL<10){
		ILI9488_DrawStringBold(106,136,b_velocity,Font32,0xFFFF);
	}

	//Sensorics
	sprintf(b_sensorics,"%d",Sensorics_Mode);
	ILI9488_DrawStringBold(446,195,"SM",Font16,0x5F00);
	ILI9488_DrawStringBold(346,134,b_sensorics,Font32,0xFFFF);

}
char tc_status[4];
char reg_status[4];
char b_laptime[20];
void screen_autox(){
	ILI9488_FillScreen_DMA(0x0000);

	ILI9488_Square(0,0,479,1,0xFFFF);//H1
	ILI9488_Square(0, 0, 1, 319, 0xFFFF);//V1
	ILI9488_Square(0, 318,479, 319, 0xFFFF);//H3
	ILI9488_Square(478,0,479,319,0xFFFF);//V3

	ILI9488_Square(239,99,240,219,0xFFFF);//V2
	ILI9488_Square(0,219,419,220,0xFFFF);//H2

	//Traction Control
	ILI9488_Square(0,0,239,99,0xF800);
	if(TC_Warning==1){strcpy(tc_status,"ON");}else if(TC_Warning==0){strcpy(tc_status,"OFF");}
	ILI9488_DrawStringBold(36,29,"TC " ,Font32,0xFFFF);
	ILI9488_DrawStringBold(120,29,tc_status,Font32,0xFFFF);


	//Regenerative
	ILI9488_Square(240,0,479,99,0x0FF0);
	if(Regen_Enabled==1){strcpy(reg_status,"ON");}else if(Regen_Enabled==0){strcpy(reg_status,"OFF");}
	ILI9488_DrawStringBold(29,40,"REGEN",Font16,0x0000);
	ILI9488_DrawStringBold(127,29,reg_status,Font32,0x0000);

	//Speed
	velocity=el_VEL*3.6;
	sprintf(b_velocity,"%d",velocity);
	ILI9488_DrawStringBold(179,195,"km/h",Font16,0x5F00);
	if(el_VEL>9){
		ILI9488_DrawStringBold(92,136,b_velocity,Font32,0xFFFF);
	}else if(el_VEL<10){
		ILI9488_DrawStringBold(106,136,b_velocity,Font32,0xFFFF);
	}

	//Sensorics Mode
	sprintf(b_sensorics,"%d",Sensorics_Mode);
	ILI9488_DrawStringBold(446,195,"SM",Font16,0x5F00);
	ILI9488_DrawStringBold(346,134,b_sensorics,Font32,0xFFFF);

	//LAP Time
	sprintf(b_laptime,"%d",LapTime);
	if(LapTime>99){
		ILI9488_DrawString(198,246,b_laptime,Font32,0xFFFF);
	}else if(LapTime>9){
		ILI9488_DrawString(212,246,b_laptime,Font32,0xFFFF);
	}else if(LapTime<10){
		ILI9488_DrawString(226,246,b_laptime,Font32,0xFFFF);
	}
	ILI9488_DrawString(302,271,"sec",Font16,0xFFFF);



}
char b_avgSpeed[20];
void screen_endurance(){
	ILI9488_FillScreen_DMA(0x0000);

	ILI9488_Square(0,0,479,1,0xFFFF);//H1
	ILI9488_Square(0, 0, 1, 319, 0xFFFF);//V1
	ILI9488_Square(0, 318,479, 319, 0xFFFF);//H3
	ILI9488_Square(478,0,479,319,0xFFFF);//V3
	ILI9488_Square(199,0,269,1,0xFFFF);
	ILI9488_Square(199,98,269,99,0xFFFF);
	ILI9488_Square(198,0,199,99,0xFFFF);
	ILI9488_Square(278,0,279,99,0xFFFF);

	ILI9488_Square(0,0,199,99,0xF800);
	ILI9488_Square(279,0,479,99,0x0CC0);
	ILI9488_Square(0,100,119,199,0xF800);//ROJO 0-25 (120px)
	ILI9488_Square(120,100,239,199,0x0F00);//AMARILLO 25-50 (120px)
	ILI9488_Square(240,100,479,199,0x07E0);//VERDE 50-100 (240px)


	ILI9488_Square(479*(SOC_Low/100),100,479,199,0xFFFF);//QUITA LINEAS
//	if(SOC_Low>49){
//		ILI9488_Square(0,100,119,199,0xF800);//ROJO 0-25
//		ILI9488_Square(120,100,239,199,0x0F00);//AMARILLO 25-50
//		ILI9488_Square(479*(SOC_Low/100),100,479,199,0xFFFF);//QUITA LINEAS
//		ILI9488_Square(240,100,479,199,0x07E0);
//		//ILI9488_Square(240,100,479*(SOC_Low/100),199,0x07E0);//VERDE 50-100
//	}else if(SOC_Low<49&&SOC_Low>24){
//		ILI9488_Square(0,100,119,199,0xF800);//ROJO 0-25
//	}
//
//	sprintf(AvgVEL_LastLap,"%d",b_avgSpeed);
	ILI9488_DrawString(54,239,"XX.X",Font32,0xFFFF);
	ILI9488_DrawString(170,261,"km/h",Font16,0x5F00);

	sprintf(b_sensorics,"%d",Sensorics_Mode);
	ILI9488_DrawString(346,239,b_sensorics,Font32,0xFFFF);
	ILI9488_DrawStringBold(374,217,"SM",Font16,0x5F00);

}
