/*
 * etr_screens.c
 *
 *  Created on: Feb 19, 2025
 *      Author: Carmen Unió Cruz
 */
#include "stm32l4xx_hal.h"
#include "DASH/etr_screens.h"
#include "LCD/ili9488.h"
#include "LCD/lcd_io_spi.h"
#include "LCD/lcd.h"
#include "LCD/stm32_adafruit_lcd.h"
#include "LCD/bmp.h"
#include "DASH/etr_carstate.h"

// Declaramos la función que dibuja la pantalla
void drawScreen(void) {
    switch (Screen.ActualState) {
        case DASH_0_ETR:
            switch (Screen.ActualScreen) {
                case SCREEN_1:
                    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
                    BSP_LCD_SetFont(&Font24);
                    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
                    BSP_LCD_Clear(LCD_COLOR_WHITE);

                    BSP_LCD_DisplayStringAt(0, 100, "CAR STATE 0", CENTER_MODE);
                	break;
                default:
                	break;
            }
            break;

        case DASH_1_PRECHARGE:
            switch (Screen.ActualScreen) {
                case SCREEN_1:
                    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
                    BSP_LCD_SetFont(&Font24);
                    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
                    BSP_LCD_Clear(LCD_COLOR_WHITE);

                    BSP_LCD_DisplayStringAt(0, 100, "PRESS OK BUTTON TO PRECHARGE", CENTER_MODE);
                    break;
                default:
                	break;
            }
            break;

        case DASH_2_PRECHARGE_STATUS:
            		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
            		BSP_LCD_SetFont(&Font24);
            		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
            		BSP_LCD_Clear(LCD_COLOR_WHITE);

            		BSP_LCD_DisplayStringAt(0, 100, "PRECHARGING...", CENTER_MODE);
            break;

        case DASH_3_PRECHARGE_FINISHED:
            		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
            		BSP_LCD_SetFont(&Font24);
            		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
            		BSP_LCD_Clear(LCD_COLOR_WHITE);

            		BSP_LCD_DisplayStringAt(0, 100, "PRECHARGE FINISHED", CENTER_MODE);
            break;

        case DASH_4_RACING_MENU:
            switch (Screen.ActualScreen) {
                case SCREEN_1:
                    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
                    BSP_LCD_SetFont(&Font24);
                    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
                    BSP_LCD_Clear(LCD_COLOR_WHITE);

                    BSP_LCD_DisplayStringAt(0, 100, "RACING MENU. PRESS OK", CENTER_MODE);
                    break;
                default:
                	break;
            }
            break;

        case DASH_5_INVERTERS:
            		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
            		BSP_LCD_SetFont(&Font24);
            		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
            		BSP_LCD_Clear(LCD_COLOR_WHITE);

            		BSP_LCD_DisplayStringAt(0, 100, "INVERTERS GETTING READY", CENTER_MODE);
            break;

        case DASH_6_RACING_MODE:
            switch (Screen.ActualScreen) {
                case SCREEN_1:
                    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
                    BSP_LCD_SetFont(&Font24);
                    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
                    BSP_LCD_Clear(LCD_COLOR_WHITE);

                    BSP_LCD_DisplayStringAt(0, 100, "VROOM VROOM", CENTER_MODE);
                    break;
                default:
                	break;
            }
            break;

        case DASH_7_ERROR:
            		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
            		BSP_LCD_SetFont(&Font24);
            		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
            		BSP_LCD_Clear(LCD_COLOR_WHITE);

            		BSP_LCD_DisplayStringAt(0, 100, "OOPSIE... SOMETHING WENT WRONG", CENTER_MODE);
            break;

        default:
            		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
            		BSP_LCD_SetFont(&Font24);
            		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
            		BSP_LCD_Clear(LCD_COLOR_WHITE);

            		BSP_LCD_DisplayStringAt(0, 100, "CAR STATE ???", CENTER_MODE);
            break;
    }
}
