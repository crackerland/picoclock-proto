#ifndef PicoPowerManager_h
#define PicoPowerManager_h

#include "PowerManager.h"
#include "PicoBattery.h"
#include "GC9A01A.h"
#include "QMI8658.h"

typedef struct PicoPowerManager
{
    PowerManager Base;
    PicoBattery Battery;
    LcdScreen* Screen;
    Qmi8658* Module;
}
PicoPowerManager;

extern void PicoPowerManager_Init(LcdScreen* screen, Qmi8658* module, PicoPowerManager* out);

#endif