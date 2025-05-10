#ifndef __ILI9488_H
#define __ILI9488_H

#include "stm32l4xx_hal.h"
#include "Dash_bitmaps/bitmaps.h"


extern SPI_HandleTypeDef hspi1;

// ----- Constants -----

//Parameters
#define BLOCK_ROWS 10
#define ILI9488_WIDTH 480
#define ILI9488_HEIGHT 320
#define PIXEL_SIZE 3// 18 bits = 3 bytes

// Nucleo PINs
#define ILI9488_CS_GPIO_Port    GPIOB
#define ILI9488_CS_Pin          GPIO_PIN_6

#define ILI9488_DC_GPIO_Port    GPIOA
#define ILI9488_DC_Pin          GPIO_PIN_8

#define ILI9488_RST_GPIO_Port   GPIOA
#define ILI9488_RST_Pin         GPIO_PIN_9

//ILI9488 commands
#define ILI9488_CMD_SOFTWARE_RESET  0x01
#define ILI9488_CMD_SLEEP_OUT       0x11
#define ILI9488_CMD_DISPLAY_ON      0x29
#define ILI9488_CMD_COLUMN_ADDR     0x2A
#define ILI9488_CMD_PAGE_ADDR       0x2B
#define ILI9488_CMD_MEMORY_WRITE    0x2C



#define CHAR_WIDTH 8
#define CHAR_HEIGHT 8

// ----- Public functions -----

//ILI9488 functions
void ILI9488_Init(void);
void ILI9488_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void ILI9488_FillScreen(uint16_t color);
void ILI9488_FillScreen_DMA(uint16_t color);
void ILI9488_Square(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
void ILI9488_DrawBitmapRGB565(uint16_t x0, uint16_t y0, uint16_t width, uint16_t height, const uint16_t *bitmap);
void ILI9488_DrawChar(uint16_t x, uint16_t y, char c, FontDef font, uint16_t color);
void ILI9488_DrawString(uint16_t x, uint16_t y, const char *str, FontDef font, uint16_t color);
//ILI9341 functions
void ILI9341_FillScreen(uint16_t color);
void ILI9341_Init(void);
void ILI9341_FillScreen_DMA(uint16_t color);




#endif
