#include "PicoTimer.h"
#include <string.h> 
#include <stdlib.h> 
#include "pico/stdlib.h"

typedef struct TimerCallbackData
{
    TimerCallback Callback;
    Timer* Timer;
    void* Param;
}
TimerCallbackData;

static int64_t OnPostComplete(alarm_id_t id, void* userData) 
{
    TimerCallbackData* callbackData = (TimerCallbackData*)userData;
    (*callbackData->Callback)(callbackData->Timer, callbackData->Param);
    free(callbackData);
    return 0;
}

static void Post(Timer* timer, uint32_t microSeconds, void* param, TimerCallback callback)
{
    TimerCallbackData* callbackData = (TimerCallbackData*)malloc(sizeof(TimerCallbackData));
    callbackData->Callback = callback;
    callbackData->Timer = timer;
    callbackData->Param = param;

    add_alarm_in_us(microSeconds, OnPostComplete, param, false);
}

static void WaitMicroseconds(Timer* timer, uint32_t us)
{
    sleep_us(us);
}

static void WaitMilliseconds(Timer* timer, uint32_t ms)
{
    sleep_ms(ms);
}

static uint32_t GetCurrentCounter(Timer* timer)
{
    // TODO
    return 0;
}

void PicoTimer_Init(PicoTimer* out)
{
    PicoTimer timer = 
    {
        .Base = 
        {
            .Post = Post,
            .WaitMicroseconds = WaitMicroseconds,
            .WaitMilliseconds = WaitMilliseconds,
            .GetCurrentCounter = GetCurrentCounter
        }
    };

    memcpy(out, &timer, sizeof(timer));
}