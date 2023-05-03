#ifndef SdlTimer_h
#define SdlTimer_h

#include "Timer.h"
#include "SDL.h"

typedef struct SdlTimer 
{
    Timer Base;
}
SdlTimer;

extern void SdlTimer_Init(SdlTimer* out);

#endif