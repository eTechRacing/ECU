/*
 * etr_carstate.h
 *
 *  Created on: Feb 25, 2025
 *      Author: cunio
 */

#ifndef INC_DASH_ETR_CARSTATE_H_
#define INC_DASH_ETR_CARSTATE_H_

#include "stdint.h"

	typedef enum {
		DASH_0_ETR,
		DASH_1_PRECHARGE,
		DASH_2_PRECHARGE_STATUS,
		DASH_3_PRECHARGE_FINISHED,
		DASH_4_RACING_MENU,
		DASH_5_INVERTERS,
		DASH_6_RACING_MODE,
		DASH_7_ERROR
	}	DASH_CarState;

	typedef enum {
		SCREEN_1,
		SCREEN_2,
		SCREEN_3,
		SCREEN_4,
		SCREEN_5,
		SCREEN_6,
		SCREEN_7,
		SCREEN_8,
		SCREEN_9,
		SCREEN_10,
		SCREEN_11,
	}	DASH_Screen;

	typedef enum {
		E1,	// FAN L ON
		E2, // FAN L OFF
		E3, // FAN L FULL
		E4, // FAN R ON
		E5, // FAN R OFF
		E6, // FAN R FULL
		E7, // PUMP R ON
		E8, // PUMP R OFF
		E9, // PUMP L ON
		E10 // PUMP L OFF
	}	DASH_Cooling;

	typedef struct {
		int L_fanStatus;
		int L_pumpStatus;
		int R_fanStatus;
		int R_pumpStatus;
		int accuRefri_status;
		int L_fan_0, L_fan_1;
		int L_pump_0, L_pump_1;
		int R_fan_0, R_fan_1;
		int R_pump_0, R_pump_1;
		int accuFan_0, accuFan_1;
	} DASH_refriSettings;

	typedef struct {
		DASH_CarState ActualState;
		DASH_CarState PreviousState;
		DASH_Screen ActualScreen;
		DASH_Screen PreviousScreen;
		DASH_Cooling CoolingState;
		int RefriMode;
		int RefriSetup;
		DASH_refriSettings refriSettings;
	}	DASH_State;

	extern int printStatus;
	extern DASH_State Screen;
	extern uint8_t CoolingRequest;

	extern const int NUM_SCREENS_PER_STATE[];
	void resetAllSignals(void);
	void refreshScreen(void);
	void drawScreen(void);
	void init_rules(void);
	void refreshGPIOs (void);
	extern uint8_t RacingMode_Send;

#endif /* INC_DASH_ETR_CARSTATE_H_ */
