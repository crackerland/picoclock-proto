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
    MotionDevice* MotionDevice;
    uint32_t LastMovementTime;
    void (*UpdateState)(struct PicoPowerManager*);
    void (*OnMotion)(struct PicoPowerManager*);
    void (*Update)(struct PicoPowerManager*);
}
PicoPowerManager;

extern void PicoPowerManager_Init(LcdScreen* screen, Qmi8658* module, MotionDevice* motionDevice, PicoPowerManager* out);

#endif