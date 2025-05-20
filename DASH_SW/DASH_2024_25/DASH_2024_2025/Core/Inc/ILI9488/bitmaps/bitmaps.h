#ifndef __bitmaps_h
#define __bitmaps_h
#include "stm32l4xx_hal.h"

typedef struct {
	uint8_t width;
	uint8_t height;
	uint8_t bytes;
	const uint8_t *table;
}FontDef;

extern const uint16_t logo[];
extern const uint16_t back[];
extern const uint16_t down[];
extern const uint16_t up[];
extern const uint16_t refri_L [];
extern const uint16_t refri_R [];
extern const uint16_t precharge_0 [];
extern const uint16_t precharge_1 [];

extern const uint8_t Font8_Table[];
extern FontDef Font8;

extern const uint8_t Font16_Table[];
extern FontDef Font16;

extern const uint8_t Font24_Table[];
extern FontDef Font24;

extern const uint8_t Font32_Table[];
extern FontDef Font32;


#endif
