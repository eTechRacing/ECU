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
                default:
                	break;
            }
            break;

        case DASH_1_PRECHARGE:
            switch (Screen.ActualScreen) {
                case SCREEN_1:
                    carState_3_SC0 ();
                    break;
                default:
                	break;
            }
            break;

        case DASH_2_PRECHARGE_STATUS:
        		carState_6();
            break;

        case DASH_3_PRECHARGE_FINISHED:
            		carState_9 ();
            break;

        case DASH_4_RACING_MENU:
            switch (Screen.ActualScreen) {
                case SCREEN_1:
                    //HERE
                    break;
                default:
                	break;
            }
            break;

        case DASH_5_INVERTERS:
            		carState_14();
            break;

        case DASH_6_RACING_MODE:
            switch (Screen.ActualScreen) {
                case SCREEN_1:
                	carState_15(0);
                    break;
                default:
                	break;
            }
            break;

        case DASH_7_ERROR:
            		carState_21 ();
            break;

        default:
            		//HERE???
            break;
    }
}
