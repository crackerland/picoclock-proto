#include "PicoDateTimeProvider.h"
#include <string.h>
#include "pico/stdlib.h"

static void GetDateTime(DateTimeProvider* provider, DateTime* out)
{
    PicoDateTimeProvider* this = (PicoDateTimeProvider*)provider;
    rtc_get_datetime(&this->Time);
    DateTime time =
    {
        .Year = this->Time.year,
        .Month = this->Time.month,
        .DayOfMonth = this->Time.day,
        .DayOfWeek = this->Time.dotw,
        .Hour = this->Time.hour,
        .Minute = this->Time.min,
        .Second = this->Time.sec,
    };

    memcpy(out, &time, sizeof(time));
}

void PicoDateTimeProvider_Init(PicoDateTimeProvider* out)
{
    PicoDateTimeProvider provider =
    {
        .Base = 
        {
            .GetDateTime = GetDateTime
        },

        // Start on Friday June 5 2020 15:45:00
        .Time = 
        {
            .year  = 2023,
            .month = 4,
            .day   = 28,
            .dotw  = 5, // 0 is Sunday, 6 is Saturday
            .hour  = 20,
            .min   = 7,
            .sec   = 00
        }
    };

    rtc_init();

    // SaveState* savedState = (SaveState*)LoadFlashState();
    // if (!strcmp(savedState->Name, "SAVE"))
    // {
    //     // Previous time was saved.
    //     datetime_t* savedTime = (datetime_t*)savedState->Content;
    //     memcpy(&provider.Time, savedTime, sizeof(datetime_t));
    // }

    rtc_set_datetime(&provider.Time);

    // clk_sys is >2000x faster than clk_rtc, so datetime is not updated immediately when rtc_get_datetime() is called.
    // tbe delay is up to 3 RTC clock cycles (which is 64us with the default clock settings)
    sleep_us(64);

    memcpy(out, &provider, sizeof(provider));
}