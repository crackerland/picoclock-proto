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
    void (*UpdateState)(struct PicoPowerManager*);
    uint32_t LastMovementTime;
    QMI8658_MotionCoordinates Acc;
    QMI8658_MotionCoordinates Gyro;
    void (*OnMotion)(struct PicoPowerManager*);
    void (*Update)(struct PicoPowerManager*);
}
PicoPowerManager;

extern void PicoPowerManager_Init(LcdScreen* screen, Qmi8658* module, PicoPowerManager* out);

#endif