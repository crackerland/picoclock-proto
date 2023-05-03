#ifndef PicoTimer_h
#define PicoTimer_h

#include "Timer.h"

typedef struct  PicoTimer
{
    Timer Base;
}
PicoTimer;

extern void PicoTimer_Init(PicoTimer* out);

#endif