#include <time.h>
#include <stdio.h>
#include "PicoClock.h"
#include "SdlScreen.h"
#include "SdlTimer.h"

static void GetDateTime(DateTimeProvider* provider, DateTime* out)
{
    time_t now = time(NULL);
    struct tm* time = localtime(&now);
    out->Year = time->tm_year + 1900; // `tm_year` is the number of years since 1900.
    out->Month = time->tm_mon;
    out->DayOfMonth = time->tm_mday;
    out->DayOfWeek = time->tm_wday;
    out->Hour = time->tm_hour;
    out->Minute = time->tm_min;
    out->Second = time->tm_sec;

    printf("Time: %lu\n", (unsigned long)now);
}

static float const Read(Battery* battery)
{
    return 4.0;
}

static Battery noOpBattery =
{
    .Read = Read
};

static Battery* GetBattery(PowerManager* pm)
{
    return &noOpBattery;
}

int main(int argc, char** argv)
{
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER);

    SdlScreen screen;
    SdlScreen_Init(&screen, 240, 240);

    SdlTimer timer;
    SdlTimer_Init(&timer);

    DateTimeProvider dateTimeProvider =
    {
        .GetDateTime = GetDateTime
    };

    PowerManager powerManager = 
    { 
        .GetBattery = GetBattery
    };

    App app;
    App_Init(&screen.Base, &timer.Base, &dateTimeProvider, &powerManager, &app);

    while (1)
    {
        SDL_Event e;
        while (SDL_PollEvent(&e))
        {
            switch (e.type)
            {
                case SDL_QUIT:
                    (*app.Lifecycle.Dispose)(&app.Lifecycle);
                    return 0;
            }
        }

        (*app.Lifecycle.Loop)(&app.Resources);
        SDL_Delay(1000);
    }

    return 0;
}