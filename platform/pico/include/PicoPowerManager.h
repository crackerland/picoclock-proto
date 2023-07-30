#ifndef PicoPowerManager_h
#define PicoPowerManager_h

#include "PicoClock.h"
#include "PowerManager.h"
#include "PicoBattery.h"
#include "GC9A01A.h"
#include "QMI8658.h"
#include "collections/LinkedList.h"

typedef struct PicoPowerManager
{
    PowerManager Base;
    PicoBattery Battery;
    LcdScreen* Screen;
    Qmi8658* Module;
    MotionDevice* MotionDevice;
    bool Sleeping;
    uint32_t LastMovementTime;
    uint8_t BacklightAtSleep;
    LinkedList SleepChangeCallbacks;
    AppPreferences* Preferences;
    InterruptCallback* WomCallback;
    void (*SetSleepTimeout)(struct PicoPowerManager*, unsigned int sleepTimeoutMillis);
    void (*SetDimTimeout)(struct PicoPowerManager*, unsigned int dimTimeoutMillis);
    void (*UpdateState)(struct PicoPowerManager*, Qmi8658_MotionVector* vector);
    void (*OnMotion)(struct PicoPowerManager*);
    void (*Update)(struct PicoPowerManager*);
}
PicoPowerManager;

extern void PicoPowerManager_Init(
    LcdScreen* screen, 
    Qmi8658* module, 
    MotionDevice* motionDevice, 
    AppPreferences* preferences,
    PicoPowerManager* out);

#endif