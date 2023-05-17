#ifndef PicoPowerManager_h
#define PicoPowerManager_h

#include "PowerManager.h"
#include "PicoBattery.h"
#include "GC9A01A.h"

typedef struct PicoPowerManager
{
    PowerManager Base;
    PicoBattery Battery;
    LcdScreen* Screen;
    uint8_t BacklightPercentageAtSleep;
}
PicoPowerManager;

extern void PicoPowerManager_Init(LcdScreen* screen, PicoPowerManager* out);

#endif