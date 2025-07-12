#include "ILI9488/UI/screen.h"
#include <stdlib.h>
#include <stdio.h>
#include "CAN/CAN_X_2025.h"
#include "CAN/CAN.h"
#include "DASH/etr_carstate.h"
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
	if(printStatus==0){
		ILI9488_FillScreen_DMA(0x0000);
		ILI9488_DrawString(137,100,"DRIVER",Font24,0xFFFF);
	}else if(printStatus==1){
		ILI9488_Square(250, 100, 267, 124, 0x0000);
	}else{
		ILI9488_DrawString(250,100,intToString(n_driver),Font24,0xFFFF);
		Driver=n_driver+1;
	}
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
	}else if (printStatus>0){
		if(Screen.RaceSettings==1){
		switch(n_race){
			case(0):
//				ILI9488_DrawStringBold(325, 5, "AUTOX", Font16, 0xFFFF); //Avoid false visual active n_race in Screen
				ILI9488_DrawStringBold(71,5,"WORKSHOP",Font16,0xF800);
				ILI9488_DrawStringBold(71,5,"WORKSHOP",Font16,0xFFFF);
				RacingMode=1;

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
void carState_21 (){
	ILI9488_FillScreen_DMA(0x0000);
	ILI9488_DrawString(170, 139, "ERROR", Font32, 0xF800);

}

