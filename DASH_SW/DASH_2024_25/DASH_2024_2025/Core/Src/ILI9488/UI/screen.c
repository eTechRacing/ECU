#include "ILI9488/UI/screen.h"
#include <stdlib.h>
#include <stdio.h>
const char* central_button = "Press Central Button to Accept";

const char* status[] = {
		"ERROR", "OK"
};
const char* ecus_list[] = {
		"ETAS", "REAR", "FRONT", "DASH", "BMS"
};

const char* sensors_list[] = {
		"APPS1&2", "ELIPSE", "SUSPENSION", "PITOT", "BRAKE PEDAL", "BRAKE PRESURE"
};

const char* shutdown_list[] = {
		"LVMS & SETAS", "BSPD & INERTIA", "SC & BOTS", "BMS", "IMD", "TSMS & TSMP", "LEFT TS", "RIGHT TS", "HV BOX", "HUD", "ACUU INTERLOCK"
};

const char* dymaic_tests[] = {
		"SKIDPAD","AUTOX", "ENDURANCE", "ACCELERATION", "WORKSHOP"
};

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
	if(decimal_part < 0) decimal_part =-decimal_part;

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
void carState_0_SC1_ecus (){
	// ----- SCREEN 2 -----
	ILI9488_FillScreen_DMA(0x0000);
	ILI9488_DrawBitmapRGB565(0,0,480,318,back);

	ILI9488_DrawBitmapRGB565(218,272,43,23,down);
	ILI9488_DrawBitmapRGB565(218,25,43,23,up);

	int y_pos = 50;
	int n_ecus = sizeof(ecus_list)/sizeof(ecus_list[0]);
	int ecus_status[n_ecus];
	ecus_status[0] = 1; //Here is ETAS state
	ecus_status[1] = 1;	//Here is REAR state
	ecus_status[2] = 0; //Here is FRONT state
	ecus_status[3] = 1;	//Here is DASH state
	ecus_status[4] = 0;	//Here is BMS state

	for (int i=0;i<n_ecus; i++){
			ILI9488_DrawString(137, y_pos, ecus_list[i], Font24, 0xFFFF);
			ILI9488_DrawString(260, y_pos, status[ecus_status[i]], Font24, 0xFFFF);
			y_pos+=30;
	}
}
//REVISAR
void carState_0_SC1_sensors (){
	// ----- SCREEN 2 -----
	ILI9488_FillScreen_DMA(0x0000);
	ILI9488_DrawBitmapRGB565(0,0,480,318,back);

	ILI9488_DrawBitmapRGB565(218,272,43,23,down);
	ILI9488_DrawBitmapRGB565(218,25,43,23,up);

	int y_pos = 50;
	int n_sens = sizeof(sensors_list)/sizeof(sensors_list[0]);
	int sens_status[n_sens];
	sens_status[0] = 1; //Here is ETAS state
	sens_status[1] = 1;	//Here is REAR state
	sens_status[2] = 0; //Here is FRONT state
	sens_status[3] = 1;	//Here is DASH state
	sens_status[4] = 0;	//Here is BMS state

	for (int i=0;i<n_sens; i++){
			ILI9488_DrawString(137, y_pos, sensors_list[i], Font24, 0xFFFF);
			ILI9488_DrawString(260, y_pos, status[sens_status[i]], Font24, 0xFFFF);
			y_pos+=30;
	}
}
//REVISAR
void carState_0_SC1_shutdown (){
	// ----- SCREEN 2 -----
	ILI9488_FillScreen_DMA(0x0000);
	ILI9488_DrawBitmapRGB565(0,0,480,318,back);

	ILI9488_DrawBitmapRGB565(218,272,43,23,down);
	ILI9488_DrawBitmapRGB565(218,25,43,23,up);

	int y_pos = 50;
	int n_sdown = sizeof(shutdown_list)/sizeof(shutdown_list[0]);
	int sdown_status[n_sdown];
	sdown_status[0] = 1;    //Here is ETAS state
	sdown_status[1] = 0;	//Here is REAR state
	sdown_status[2] = 1;    //Here is FRONT state
	sdown_status[3] = 1;	//Here is DASH state
	sdown_status[4] = 0;	//Here is BMS state

	for (int i=0;i<n_sdown; i++){
			ILI9488_DrawString(137, y_pos, shutdown_list[i], Font24, 0xFFFF);
			ILI9488_DrawString(260, y_pos, status[sdown_status[i]], Font24, 0xFFFF);
			y_pos+=30;
	}
}
//MANDOS CENTRALES
void carState_0_SC2 (){
	// ----- SCREEN 3 -----

	ILI9488_FillScreen_DMA(0x0000);
	ILI9488_DrawString(60, 10, "REFRI CONFIGURATION", Font24, 0xFFFF);
	ILI9488_DrawBitmapRGB565(25,50,120,220,refri_L);
	ILI9488_Square(76.2-25-1, 248.5+3, 76.8+76.2-25-6+1, 248.5+9.9+3, 0x07E0);
	//ILI9488_Square(50, 50, 170, 270, 0xf006);
	//ILI9488_Square(180, 57, 300, 107, 0x6629);
	ILI9488_DrawBitmapRGB565(335,50,120,220,refri_R);
	ILI9488_Square(333.3+24, 248.5, 429.1, 248.5+9.9, 0x07E0);
	ILI9488_Square(180, 134, 300, 184, 0x6629);
	ILI9488_Square(180, 211, 300, 261, 0xCB19);
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

void carState_3_SC1 (){
	// ----- SCREEN 2 -----
	ILI9488_FillScreen_DMA(0x0000);
}

void carState_3_SC2 (){
	// ----- SCREEN 2 -----
	ILI9488_FillScreen_DMA(0x0000);
}

/////////////////////////
//----- CAR_STATE - 6
////////////////////////
void carState_6 (int status){
	ILI9488_FillScreen_DMA(0x0000);
		ILI9488_DrawBitmapRGB565(102.5,30,275,110,precharge_1);
		int i = 0;
		int x = 46;
		int y = 197;
		/*
		if (status<i){
			ILI9488_Square(x, y, x+45, y+30, 0x07E0);
			x+=49;
			i+=10;
			HAL_Delay(500);
		}
		*/
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
	if(column == 0){
		//FOR THE RACE TYPE
	}else if(column == 1){
		//FOR THE DRIVER SELECT
	}else if(column == 2){
		//FOR THE FINAL SCREEN
	}
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
	ILI9488_DrawString(137, 100, "ERROR SOMETHING WENT RONG", Font24, 0xFFFF);
}
