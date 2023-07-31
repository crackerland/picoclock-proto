#ifndef PicoPreferenceManager_h
#define PicoPreferenceManager_h

#include "PreferenceManager.h"

typedef struct PicoPreferenceManager
{
    PreferenceManager Base;
}
PicoPreferenceManager;

extern void PicoPreferenceManager_Init(PicoPreferenceManager* out);

#endif