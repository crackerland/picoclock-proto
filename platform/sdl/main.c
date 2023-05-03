#include <time.h>
#include "PicoClock.h"
#include "SdlScreen.h"
#include "SdlTimer.h"

static void GetDateTime(DateTimeProvider* provider, DateTime* out)
{
    // time_t rawTime;
    // time(&rawTime);
    // struct tm* timeInfo = localtime(&rawTime);
    time_t now = time(NULL);
    out->Year = 2023;
    out->Month = 5;
    out->DayOfMonth = 3;
    out->DayOfWeek = 3;
    out->Hour = 13;
    out->Minute = 30;
    out->Second = 25;
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

    PicoClock_Start(&screen.Base, &timer.Base, &dateTimeProvider);

    return 0;
}