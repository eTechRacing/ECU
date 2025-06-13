#ifndef __screen_h
#define __screen_h

#include "ILI9488/bitmaps/bitmaps.h"
#include "ILI9488/lib/mainDash_lib.h"

//FUNCTION DECLARATION
void carState_0_SC0 (void);
void carState_0_SC1_ecus (int mode);
void carState_0_SC1_sensors (void);
void carState_0_SC1_shutdown (void);
void carState_0_SC2_refri (void);

void carState_3_SC0 (void);

void carState_6 (void);

void carState_9 (void);

void carState_12_DYNAMIC (int screen, int column);
void carState_12_DRIVER (int screen, int column);

void carState_14 (void);

void carState_15 (int n_race);

void carState_21 (void);

#endif
