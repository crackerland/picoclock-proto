#include "SdlTimer.h"
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

typedef struct TimerCallbackArgs
{
    Timer* Timer;
    TimerCallback Callback;
    void* Param;
    SDL_TimerID TimerId;
}
TimerCallbackArgs;

static uint32_t OnTimerComplete(uint32_t interval, void* param)
{
    TimerCallbackArgs* args = (TimerCallbackArgs*)param;
    (*args->Callback)((struct Timer*)args->Timer, args->Param);
    SDL_RemoveTimer(args->TimerId);
    free(args);
    return 0;
}

static void Post(Timer* timer, uint32_t microSeconds, void* param, TimerCallback callback)
{
    TimerCallbackArgs* args = malloc(sizeof(TimerCallbackArgs));
    args->Callback = callback;
    args->Param = param;
    args->Timer = timer;
    args->TimerId = SDL_AddTimer(microSeconds / 1000, OnTimerComplete, args);
}

static void WaitMicroseconds(Timer* timer, uint32_t us)
{
    usleep(us);
}

static void WaitMilliseconds(Timer* timer, uint32_t ms)
{
    SDL_Delay(ms);
}

static uint32_t GetCurrentCounter(Timer* timer)
{
    return 0;
}

void SdlTimer_Init(SdlTimer* out)
{
    SdlTimer timer =
    {
        .Base = 
        {
            .Post = Post,
            .WaitMicroseconds = WaitMicroseconds,
            .WaitMilliseconds = WaitMilliseconds,
            .GetCurrentCounter = GetCurrentCounter
        }
    };

    memcpy(out, &timer, sizeof(SdlTimer));
}