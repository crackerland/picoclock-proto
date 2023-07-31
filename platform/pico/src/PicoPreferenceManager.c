#include "PicoPreferenceManager.h"
#include "PicoPersistentStorage.h"
#include <string.h>

#define FLASH_KEY "PREFS"

static void Save(PreferenceManager* manager, AppPreferences* preferences)
{
    SaveFlashState(FLASH_KEY, preferences, sizeof(preferences));
}

static void Load(PreferenceManager* manager, AppPreferences* out)
{
    if (LoadFlashState(FLASH_KEY, out, sizeof(AppPreferences)))
    {
        // Existing flash state found.
        return;
    }

    // No persistent state found. Return default.
    AppPreferences defaults = 
    {
        .SleepTimeoutMillis = 60 * 1000, // 1 minute.
        .DimTimeoutMillis = 7 * 1000, // 7 seconds.
        .FullBrightness = 90,
        .DimBrightness = 5
    };

    memcpy(out, &defaults, sizeof(AppPreferences));
}

void PicoPreferenceManager_Init(PicoPreferenceManager* out)
{
    PicoPreferenceManager manager =
    {
        .Base = 
        {
            .Load = Load,
            .Save = Save
        }
    };

    memcpy(out, &manager, sizeof(PicoPreferenceManager));
}