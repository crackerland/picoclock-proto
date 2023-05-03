#ifndef PicoDateTimeProvider_h 
#define PicoDateTimeProvider_h 

#include "DateTime.h"
#include "hardware/rtc.h"
#include "pico/util/datetime.h"

typedef struct PicoDateTimeProvider
{
    DateTimeProvider Base;
    datetime_t Time;
}
PicoDateTimeProvider;

extern void PicoDateTimeProvider_Init(PicoDateTimeProvider* out);

#endif