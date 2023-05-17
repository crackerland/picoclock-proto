#include "PicoPowerManager.h"
#include <string.h>
#include "QMI8658.h"

static void Sleep(PowerManager* powerManager)
{
    PicoPowerManager* this = (PicoPowerManager*)powerManager;
    this->BacklightPercentageAtSleep = this->Screen->CurrentBacklightPercentage;
    (*this->Screen->Base.SetBacklightPercentage)(&this->Screen->Base, 0);
    (*this->Screen->SetSleep)(this->Screen, true);

    // QMI8658_enableWakeOnMotion();
    // while (1)
    // {
    //     uint8_t b;
    //     QMI8658_read_reg(QMI8658Register_Status1, &b, 1);

    //     if (b == QMI8658_STATUS1_WAKEUP_EVENT)
    //     {
    //         break;
    //     }

    //     sleep_ms(250);
    // }

    // QMI8658_disableWakeOnMotion();
    // sleep_ms(10);
    // QMI8658_reenable();
}

static void WakeUp(PowerManager* powerManager)
{
    PicoPowerManager* this = (PicoPowerManager*)powerManager;
    (*this->Screen->Base.SetBacklightPercentage)(&this->Screen->Base, this->BacklightPercentageAtSleep);
    (*this->Screen->SetSleep)(this->Screen, false);
}

static Battery* GetBattery(PowerManager* manager)
{
    PicoPowerManager* this = (PicoPowerManager*)manager;
    return &this->Battery.Base;
}

void PicoPowerManager_Init(LcdScreen* screen, PicoPowerManager* out)
{
    PicoPowerManager manager =
    {
        .Base = 
        {
            .Sleep = Sleep,
            .WakeUp = WakeUp,
            .GetBattery = GetBattery
        },
        .Screen = screen
    };

    memcpy(out, &manager, sizeof(manager));

    PicoBattery_Init(1000.0f, &out->Battery);
}