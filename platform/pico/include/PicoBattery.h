#ifndef PicoBattery_h
#define PicoBattery_h

#include "Battery.h"

typedef struct PicoBattery
{ 
    Battery Base;
    char mode[8];
    float mA;
    float load;
    float max;
    float min;
    float dif;
    float read;
} 
PicoBattery;

extern void PicoBattery_Init(float batteryMah, PicoBattery* out);

#endif