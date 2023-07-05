#include "SystemTimerDateTimeProvider.h"
#include <string.h>

static void GetDateTime(DateTimeProvider* provider, DateTime* out)
{
    out->DayOfMonth = 1;
    out->DayOfWeek = 1;
    out->Hour = 10;
    out->Minute = 35;
    out->Second = 23;
    out->Year = 1991;
}

void SystemTimerDateTimeProvider_Init(Timer* timer, SystemTimerDateTimeProvider* out)
{
    // uint32_t time = (*timer->GetCurrentCounter)(timer);
    SystemTimerDateTimeProvider provider = 
    {
        .Base = 
        {
            .GetDateTime = GetDateTime
        },
        .Timer = timer
    };

    memcpy(out, &provider, sizeof(SystemTimerDateTimeProvider));
}