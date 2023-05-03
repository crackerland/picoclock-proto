#ifndef PicoClock_h
#define PicoClock_h

#include "Screen.h"
#include "Timer.h"
#include "DateTime.h"

extern void PicoClock_Start(    
    Screen* screen, 
    Timer* timer, 
    DateTimeProvider* dateTimeProvider);

#endif