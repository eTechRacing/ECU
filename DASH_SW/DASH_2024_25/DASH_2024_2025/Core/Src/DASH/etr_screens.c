/*
 * etr_screens.c
 *
 *  Created on: Feb 19, 2025
 *      Author: Carmen Unió Cruz
 */
#include "stm32l4xx_hal.h"
#include "DASH/etr_screens.h"
#include "ILI9488/UI/screen.h"
#include "DASH/etr_carstate.h"

// Declaramos la función que dibuja la pantalla
void drawScreen(void) {
    switch (Screen.ActualState) {
        case DASH_0_ETR:
            switch (Screen.ActualScreen) {
                case SCREEN_1:
                	carState_0_SC0 ();
                	break;
            }
            break;

        case DASH_1_PRECHARGE:
            switch (Screen.ActualScreen) {
                case SCREEN_1:
                    carState_3_SC0 ();
                    break;
            }
            break;

        case DASH_2_PRECHARGE_STATUS:
            switch (Screen.ActualScreen) {
            	case SCREEN_1:
            		carState_6();
            		break;
            }
            break;

        case DASH_3_PRECHARGE_FINISHED:
            switch (Screen.ActualScreen) {
            	case SCREEN_1:
            		carState_9 ();
            		break;
            }
            break;

        case DASH_4_RACING_MENU:
            switch (Screen.ActualScreen) {
                case SCREEN_1:
                    //HERE
                    break;
            }
            break;

        case DASH_5_INVERTERS:
            switch (Screen.ActualScreen) {
                case SCREEN_1:
            		carState_14();
            		break;
            }
            break;

        case DASH_6_RACING_MODE:
            switch (Screen.ActualScreen) {
                case SCREEN_1:
                	carState_15(0);
                	break;
            }
            break;

        case DASH_7_ERROR:
            switch (Screen.ActualScreen) {
                case SCREEN_1:
            		carState_21 ();
            		break;
            }
            break;


    }
}
