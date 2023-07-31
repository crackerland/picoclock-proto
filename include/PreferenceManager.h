#ifndef PreferenceManager_h
#define PreferenceManager_h

#include <stdint.h>

typedef struct AppPreferences
{
    unsigned int SleepTimeoutMillis;
    unsigned int DimTimeoutMillis;
    uint8_t FullBrightness;
    uint8_t DimBrightness;
}
AppPreferences;

typedef struct PreferenceManager
{
    void (*Save)(struct PreferenceManager*, AppPreferences* preferences);
    void (*Load)(struct PreferenceManager*, AppPreferences* out);
}
PreferenceManager;

#endif