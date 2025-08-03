/*************************************************************
 * Project: Dashboard Library - ETECH Racing FS team
 * File:   mainDash_lib.c
 * Author:     Maximo028
 * Role:       ECUS dept.
 * DATE:     20/05/2025
 * Description (READ CAREFULY BEFORE ANY USAGE):
 *   Dashboard library for the init & diff functions of the ILI9488 LCD Screen 480x320 px
 *	 NOTE: the ILI9488 works with RGB666, all the data must be RGB565 bc all the functions have a converter
 * License: See Below
 *************************************************************/
#include <stdint.h>

#include "ILI9488/lib/mainDash_lib.h"


volatile uint8_t dma_done = 0;
static uint8_t dma_buffer[ILI9488_WIDTH * PIXEL_SIZE * BLOCK_ROWS];

//DMA - Callback
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi == &hspi3) {
        dma_done = 1;
    }
}
// Private function to send command
static void ILI9488_SendCommand(uint8_t cmd) {
    HAL_GPIO_WritePin(ILI9488_DC_GPIO_Port, ILI9488_DC_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ILI9488_CS_GPIO_Port, ILI9488_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, &cmd, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ILI9488_CS_GPIO_Port, ILI9488_CS_Pin, GPIO_PIN_SET);
}

// Private function to send data
static void ILI9488_SendData(uint8_t data) {
    HAL_GPIO_WritePin(ILI9488_DC_GPIO_Port, ILI9488_DC_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ILI9488_CS_GPIO_Port, ILI9488_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, &data, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ILI9488_CS_GPIO_Port, ILI9488_CS_Pin, GPIO_PIN_SET);
}

// SW Reset (NSS must be in - SW mode)
static void ILI9488_Reset(void) {
    HAL_GPIO_WritePin(ILI9488_RST_GPIO_Port, ILI9488_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(ILI9488_RST_GPIO_Port, ILI9488_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(150);
}

// Init Sequence
void ILI9488_Init(void) {
	 ILI9488_Reset();
	    HAL_Delay(10);

	    ILI9488_SendCommand(0x11); // Sleep out
	    HAL_Delay(120);

	    ILI9488_SendCommand(0x3A); // Pixel format
	    ILI9488_SendData(0x66);    // 18 bits per pixel (RGB666)

	    ILI9488_SendCommand(0x36); // MADCTL
	    ILI9488_SendData(0x28);    // Landscape + BGR (0xE8 or 0x28 for horizontal view depending on the side)

	    ILI9488_SendCommand(0x29); // Display on
	    HAL_Delay(20);
}

// Private function to set window
static void ILI9488_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    ILI9488_SendCommand(ILI9488_CMD_COLUMN_ADDR);
    ILI9488_SendData(x0 >> 8);
    ILI9488_SendData(x0 & 0xFF);
    ILI9488_SendData(x1 >> 8);
    ILI9488_SendData(x1 & 0xFF);

    ILI9488_SendCommand(ILI9488_CMD_PAGE_ADDR);
    ILI9488_SendData(y0 >> 8);
    ILI9488_SendData(y0 & 0xFF);
    ILI9488_SendData(y1 >> 8);
    ILI9488_SendData(y1 & 0xFF);

    ILI9488_SendCommand(ILI9488_CMD_MEMORY_WRITE);
}

// Draw a single pixel (avoid the use for large data, it is very very slow for big volum of points)
void ILI9488_DrawPixel(uint16_t x, uint16_t y, uint16_t color) {
    // RGB565 to RGB666
    uint8_t r = ((color >> 11) & 0x1F) << 1;
    uint8_t g = ((color >> 5) & 0x3F);
    uint8_t b = ((color & 0x1F) << 1);


    r = r << 2;
    g = g << 2;
    b = b << 2;

    ILI9488_SetWindow(x, y, x, y);

    HAL_GPIO_WritePin(ILI9488_DC_GPIO_Port, ILI9488_DC_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ILI9488_CS_GPIO_Port, ILI9488_CS_Pin, GPIO_PIN_RESET);

    uint8_t data[3] = {r, g, b};
    HAL_SPI_Transmit(&hspi3, data, 3, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(ILI9488_CS_GPIO_Port, ILI9488_CS_Pin, GPIO_PIN_SET);
}

//Fill all the screen with a single color && DMA for a CPU better flow
void ILI9488_FillScreen_DMA(uint16_t color) {
    uint8_t r = ((color >> 11) & 0x1F) << 1;
    uint8_t g = ((color >> 5) & 0x3F);
    uint8_t b = ((color & 0x1F) << 1);

    r = r << 2;
    g = g << 2;
    b = b << 2;

    for (int i = 0; i < ILI9488_WIDTH * BLOCK_ROWS; i++) {
        dma_buffer[i * 3 + 0] = r;
        dma_buffer[i * 3 + 1] = g;
        dma_buffer[i * 3 + 2] = b;
    }

    for (int y = 0; y < ILI9488_HEIGHT; y += BLOCK_ROWS) {
        ILI9488_SetWindow(0, y, ILI9488_WIDTH - 1, y + BLOCK_ROWS - 1);

        HAL_GPIO_WritePin(ILI9488_DC_GPIO_Port, ILI9488_DC_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(ILI9488_CS_GPIO_Port, ILI9488_CS_Pin, GPIO_PIN_RESET);

        dma_done = 0;
        HAL_SPI_Transmit_DMA(&hspi3, dma_buffer, sizeof(dma_buffer));
        while (!dma_done);

        HAL_GPIO_WritePin(ILI9488_CS_GPIO_Port, ILI9488_CS_Pin, GPIO_PIN_SET);
    }
}

//Draw a square
static uint8_t rect_buffer[ILI9488_WIDTH * PIXEL_SIZE];
void ILI9488_Square(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color){
	    if (x1 < x0 || y1 < y0 || x1 >= ILI9488_WIDTH || y1 >= ILI9488_HEIGHT) return;

	    uint8_t r = ((color >> 11) & 0x1F) << 1;
	    uint8_t g = ((color >> 5) & 0x3F);
	    uint8_t b = ((color & 0x1F) << 1);

	    r <<= 2; g <<= 2; b <<= 2;

	    uint16_t width = x1 - x0 + 1;
	    uint16_t height = y1 - y0 + 1;

	    for (int i = 0; i < width; i++) {
	        rect_buffer[i * 3 + 0] = r;
	        rect_buffer[i * 3 + 1] = g;
	        rect_buffer[i * 3 + 2] = b;
	    }

	    ILI9488_SetWindow(x0, y0, x1, y1);

	    HAL_GPIO_WritePin(ILI9488_DC_GPIO_Port, ILI9488_DC_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(ILI9488_CS_GPIO_Port, ILI9488_CS_Pin, GPIO_PIN_RESET);

	    for (int y = 0; y < height; y++) {
	        HAL_SPI_Transmit(&hspi3, rect_buffer, width * 3, HAL_MAX_DELAY);
	    }

	    HAL_GPIO_WritePin(ILI9488_CS_GPIO_Port, ILI9488_CS_Pin, GPIO_PIN_SET);
}
//Draw a bitmap
void ILI9488_DrawBitmapRGB565(uint16_t x0, uint16_t y0, uint16_t width, uint16_t height, const uint16_t *bitmap) {
	 if (x0 + width > ILI9488_WIDTH || y0 + height > ILI9488_HEIGHT) return;

	    ILI9488_SetWindow(x0, y0, x0 + width - 1, y0 + height - 1);

	    HAL_GPIO_WritePin(ILI9488_DC_GPIO_Port, ILI9488_DC_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(ILI9488_CS_GPIO_Port, ILI9488_CS_Pin, GPIO_PIN_RESET);

	    uint8_t pixel_buffer[3];

	    for (uint32_t i = 0; i < width * height; i++) {
	        uint16_t color = bitmap[i];

	        uint8_t r = ((color >> 11) & 0x1F) << 1;
	        uint8_t g = ((color >> 5) & 0x3F);
	        uint8_t b = ((color & 0x1F) <<1);

	        r <<= 2;
	        g <<= 2;
	        b <<= 2;

	        pixel_buffer[0] = b;
	        pixel_buffer[1] = g;
	        pixel_buffer[2] = r;

	        HAL_SPI_Transmit(&hspi3, pixel_buffer, 3, HAL_MAX_DELAY);
	    }

	    HAL_GPIO_WritePin(ILI9488_CS_GPIO_Port, ILI9488_CS_Pin, GPIO_PIN_SET);
}
//Draw a single char
void ILI9488_DrawChar(uint16_t x, uint16_t y, char c, FontDef font, uint16_t color){
	uint32_t offset = (c - 32) * font.height * font.bytes;
	    const uint8_t *bitmap = &font.table[offset];

	    for (uint8_t row = 0; row < font.height; row++) {
	        uint64_t line = 0;

	        for (uint8_t b = 0; b < font.bytes; b++) {
	            line <<= 8;
	            line |= bitmap[row * font.bytes + b];
	        }

	        line = line >> ((font.bytes * 8) - font.width); // Alinear a la derecha
	        line &= (1 << font.width) - 1; // Limpiar bits sobrantes

	        for (uint8_t col = 0; col < font.width; col++) {
	            if (line & (1 << (font.width - 1 - col))) {
	                ILI9488_DrawPixel(x + col, y + row, color);
	            }
	        }
	    }

}

void ILI9488_DrawCharBold(uint16_t x, uint16_t y, char c, FontDef font, uint16_t color) {
    uint32_t offset = (c - 32) * font.height * font.bytes;
    const uint8_t *bitmap = &font.table[offset];

    for (uint8_t row = 0; row < font.height; row++) {
        uint64_t line = 0;

        // Leer todos los bytes de la fila
        for (uint8_t b = 0; b < font.bytes; b++) {
            line <<= 8;
            line |= bitmap[row * font.bytes + b];
        }

        line = line >> ((font.bytes * 8) - font.width);
        line &= (1 << font.width) - 1;

        for (uint8_t col = 0; col < font.width; col++) {
            if (line & (1 << (font.width - 1 - col))) {
                ILI9488_DrawPixel(x + col, y + row, color);
                if (col < font.width - 1 && !(line & (1 << (font.width - 2 - col)))) {
                    ILI9488_DrawPixel(x + col + 1, y + row, color); // Efecto bold: pixel adicional a la derecha
                }
            }
        }
    }
}
//Draw a string
void ILI9488_DrawString(uint16_t x, uint16_t y, const char *str, FontDef font, uint16_t color){
	uint16_t x_start = x;
	while (*str) {
		if(*str == '\n'){
			y += font.height;
			x = x_start;
			str++;
			continue;
		}
		if((x + font.width) > ILI9488_WIDTH){
			y += font.height;
			x = x_start;
		}
		if((y + font.height) > ILI9488_HEIGHT){
			break;
		}
		ILI9488_DrawChar(x, y, *str, font, color);
		x += font.width;
		str++;
	    }
}

void ILI9488_DrawStringBold(uint16_t x, uint16_t y, const char *str, FontDef font, uint16_t color) {
    uint16_t x_start = x;
    while (*str) {
        if (*str == '\n') {
            y += font.height;
            x = x_start;
            str++;
            continue;
        }
        if ((x + font.width + 1) > ILI9488_WIDTH) {
            y += font.height;
            x = x_start;
        }
        if ((y + font.height) > ILI9488_HEIGHT) {
            break;
        }

        ILI9488_DrawCharBold(x, y, *str, font, color);
        x += font.width + 1; // Compensamos el píxel extra del bold
        str++;
    }
}

void ILI9488_FillCircle(uint16_t x0, uint16_t y0, uint16_t radius, uint16_t color) {
    for (int y = -radius; y <= radius; y++) {
        for (int x = -radius; x <= radius; x++) {
            if (x * x + y * y <= radius * radius) {
                ILI9488_DrawPixel(x0 + x, y0 + y, color);
            }
        }
    }
}

void ILI9488_DrawBitmapMono(uint16_t x, uint16_t y, const uint8_t *bitmap, uint16_t width, uint16_t height, uint16_t color) {
    uint8_t bytesPerRow = (width + 7) / 8;

    for (uint16_t row = 0; row < height; row++) {
        for (uint16_t col = 0; col < width; col++) {
            uint16_t byteIndex = row * bytesPerRow + col / 8;
            uint8_t bitMask = 0x80 >> (col % 8);

            if (bitmap[byteIndex] & bitMask) {
                ILI9488_DrawPixel(x + col, y + row, color);
            }
        }
    }
}
/* LICENSE: MIT
 * Code for ETECH RACING FS team internal use.
 * Can be used, modified and redistribuited inside the team
 * with academic and competitive prouposes.
 * External contributors must retain this notice.
 */
