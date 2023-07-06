#include "PicoPowerManager.h"
#include <string.h>
#include "QMI8658.h"

static void ShutDown(PowerManager* powerManager)
{
}

static void Sleep(PowerManager* powerManager)
{
    PicoPowerManager* this = (PicoPowerManager*)powerManager;

    uint8_t backlight = this->Screen->CurrentBacklightPercentage;
    (*this->Screen->Base.SetBacklightPercentage)(&this->Screen->Base, 0);
    (*this->Screen->SetSleep)(this->Screen, true);

    QMI8658_enableWakeOnMotion(this->Module);
    while (this->Module->Sleeping)
    {
        sleep_ms(250);
    }

    QMI8658_disableWakeOnMotion(this->Module);
    sleep_ms(10);
    QMI8658_reenable();

    // Acknowledge the wake up interrupt and clear the status bit by reading.
    // TODO: This should probably be done in the module API.
    uint8_t status;
    QMI8658_read_reg(QMI8658Register_Status1, &status, 1);
    if (status & QMI8658_STATUS1_WAKEUP_EVENT)
    {
    }

    (*this->Screen->SetSleep)(this->Screen, false);
    (*this->Screen->Base.SetBacklightPercentage)(&this->Screen->Base, backlight);
}

static void WakeUp(PowerManager* powerManager)
{
    PicoPowerManager* this = (PicoPowerManager*)powerManager;
    // (*this->Screen->Base.SetBacklightPercentage)(&this->Screen->Base, this->BacklightPercentageAtSleep);
    // (*this->Screen->SetSleep)(this->Screen, false);
}

static Battery* GetBattery(PowerManager* manager)
{
    PicoPowerManager* this = (PicoPowerManager*)manager;
    return &this->Battery.Base;
}

void PicoPowerManager_Init(LcdScreen* screen, Qmi8658* module, PicoPowerManager* out)
{
    PicoPowerManager manager =
    {
        .Base = 
        {
            .Sleep = Sleep,
            .WakeUp = WakeUp,
            .GetBattery = GetBattery, 
            .ShutDown = ShutDown
        },
        .Screen = screen,
        .Module = module
    };

    memcpy(out, &manager, sizeof(manager));

    PicoBattery_Init(1000.0f, &out->Battery);
}