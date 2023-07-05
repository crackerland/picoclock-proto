#ifndef SystemTimerDateTimeProvider_h
#define SystemTimerDateTimeProvider_h

#include "DateTime.h"
#include "Timer.h"

typedef struct SystemTimerDateTimeProvider
{
    DateTimeProvider Base;
    Timer* Timer;
}
SystemTimerDateTimeProvider;

extern void SystemTimerDateTimeProvider_Init(Timer* timer, SystemTimerDateTimeProvider* out);

#endif