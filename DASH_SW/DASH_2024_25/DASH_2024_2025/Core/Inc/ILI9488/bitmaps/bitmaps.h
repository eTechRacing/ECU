#ifndef __bitmaps_h
#define __bitmaps_h
#include "stm32l4xx_hal.h"

typedef struct {
	uint8_t width;
	uint8_t height;
	uint8_t bytes;
	const uint8_t *table;
}FontDef;

extern const uint8_t logo2[];
extern const uint16_t logo[];
extern const uint8_t skidpad[];
extern const uint8_t autox[];
extern const uint8_t endurance[];
extern const uint8_t acceleration[];


extern const uint8_t Font8_Table[];
extern FontDef Font8;

extern const uint8_t Font16_Table[];
extern FontDef Font16;

extern const uint8_t Font24_Table[];
extern FontDef Font24;

extern const uint8_t Font32_Table[];
extern FontDef Font32;


#endif
