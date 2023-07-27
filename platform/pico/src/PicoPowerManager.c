#include "PicoPowerManager.h"
#include <string.h>
#include "QMI8658.h"
#include "hardware/vreg.h"

typedef struct SleepChangeCallback
{
    SleepChangeCallbackHandler OnSleepChanged;
    PicoPowerManager* PowerManager;
    void* Payload;
}
SleepChangeCallback;

static void UpdateStateNormal(PicoPowerManager*);

static inline void NotifySleepStateChanged(PicoPowerManager* this)
{
    Iterable* callbacks = (*this->SleepChangeCallbacks.Base.GetIterable)(&this->SleepChangeCallbacks.Base);
    Iterator* next = (*callbacks->Start)(callbacks);
    while (next)
    {
        SleepChangeCallback* callback = (*next->GetValue)(next);
        (*callback->OnSleepChanged)(&this->Base, false, callback->Payload);
        next = (*next->GetNext)(next);
    }
}

static void OnWake(void* payload)
{
    PicoPowerManager* this = (PicoPowerManager*)payload;
    this->Sleeping = false;

    // vreg_set_voltage(VREG_VOLTAGE_DEFAULT);
    (*this->Module->DisableWakeOnMotion)(this->Module);

    (*this->Screen->SetSleep)(this->Screen, false);
    (*this->Screen->Base.SetBacklightPercentage)(&this->Screen->Base, this->BacklightAtSleep);

    NotifySleepStateChanged(this);
}

static void ShutDown(PowerManager* powerManager)
{
    PicoPowerManager* this = (PicoPowerManager*)powerManager;
    (*this->Module->PowerDown)(this->Module);
}

static void Sleep(PowerManager* powerManager)
{
    PicoPowerManager* this = (PicoPowerManager*)powerManager;
    this->Sleeping = true;
    this->BacklightAtSleep = this->Screen->CurrentBacklightPercentage;
    (*this->Screen->Base.SetBacklightPercentage)(&this->Screen->Base, 0);
    (*this->Screen->SetSleep)(this->Screen, true);

    (*this->Module->EnableWakeOnMotion)(this->Module, OnWake, this);

    NotifySleepStateChanged(this);

    // This causes a reboot.
    // vreg_set_voltage(VREG_VOLTAGE_MIN);
}

static void WakeUp(PowerManager* powerManager)
{
    PicoPowerManager* this = (PicoPowerManager*)powerManager;
}

static void Reset(PowerManager* powerManager)
{
    PicoPowerManager* this = (PicoPowerManager*)powerManager;
}

static Battery* GetBattery(PowerManager* manager)
{
    PicoPowerManager* this = (PicoPowerManager*)manager;
    return &this->Battery.Base;
}

static void OnMotion(PicoPowerManager* state)
{
    state->LastMovementTime = time_us_32();
    (*state->Screen->Base.SetBacklightPercentage)(&state->Screen->Base, 90);
    state->UpdateState = UpdateStateNormal;
}

static void UpdateStateDim(PicoPowerManager* state)
{
    uint32_t ellapsedTime = (time_us_32() - state->LastMovementTime) / 1000;

    if (ellapsedTime > state->Preferences->SleepTimeoutMillis)
    {
        (*state->Base.Sleep)(&state->Base);
        (*state->OnMotion)(state);
    }
}

static void UpdateStateNormal(PicoPowerManager* state)
{
    uint32_t ellapsedTime = (time_us_32() - state->LastMovementTime) / 1000;

    if (ellapsedTime > state->Preferences->DimTimeoutMillis)
    {
        (*state->Screen->Base.SetBacklightPercentage)(&state->Screen->Base, 5);
        state->UpdateState = UpdateStateDim;
    }
}

static void Update(PicoPowerManager* powerManager)
{
    // QMI8658_read_xyz(&powerManager->Acc, &powerManager->Gyro);
    // QMI8658_read_gyro_xyz(&powerManager->Gyro);
    // QMI8658_read_acc_xyz(&powerManager->Acc);
    // printf("ACC (%f, %f, %f)\n", powerManager->Acc.X, powerManager->Acc.Y, powerManager->Acc.Z);
    // (*powerManager->Module->Gyroscope.Read)(&powerManager->Module->Gyroscope, &powerManager->Gyro);
    // (*powerManager->Module->Accelerometer.Read)(&powerManager->Module->Accelerometer, &powerManager->Acc);

    // (*powerManager->Module->ReadCombined)(
    //     powerManager->Module, 
    //     &powerManager->Acc,
    //     &powerManager->Gyro);

    // printf(
    //     "ACC (%f, %f, %f) GYR (%f, %f, %f)\n", 
    //     powerManager->Acc.X, 
    //     powerManager->Acc.Y, 
    //     powerManager->Acc.Z, 
    //     powerManager->Gyro.X, 
    //     powerManager->Gyro.Y, 
    //     powerManager->Gyro.Z);

    Qmi8658_MotionVector vector;
    Qmi8658_Quaternion quaternion;
    (*powerManager->MotionDevice->Read)(powerManager->MotionDevice, &vector, &quaternion);

    // printf(
    //     "Vector (%f, %f, %f) Quaternion (%f, %f, %f, %f)\n", 
    //     vector.X, 
    //     vector.Y, 
    //     vector.Z,
    //     quaternion.W,
    //     quaternion.X,
    //     quaternion.Y,
    //     quaternion.Z);

#define GYRMAX 300.0f
// #define ACCMAX 500.0f

    if (vector.X > GYRMAX || vector.X < -GYRMAX) 
    {
        (*powerManager->OnMotion)(powerManager);
    }
    else
    {
        (*powerManager->UpdateState)(powerManager);
    }
}

static void AddSleepChangeCallback(PowerManager* powerManager, SleepChangeCallbackHandler onChange, void* payload)
{
    PicoPowerManager* this = (PicoPowerManager*)powerManager;
    SleepChangeCallback* callback = (SleepChangeCallback*)malloc(sizeof(SleepChangeCallback));
    callback->Payload = payload;
    callback->PowerManager = this;
    callback->OnSleepChanged = onChange;
    (*this->SleepChangeCallbacks.Base.Add)(&this->SleepChangeCallbacks.Base, callback);
}

void PicoPowerManager_Init(
    LcdScreen* screen, 
    Qmi8658* module, 
    MotionDevice* motionDevice, 
    AppPreferences* preferences,
    PicoPowerManager* out)
{
    PicoPowerManager manager =
    {
        .Base = 
        {
            .Sleep = Sleep,
            .WakeUp = WakeUp,
            .ShutDown = ShutDown,
            .Reset = Reset,
            .AddSleepChangeCallback = AddSleepChangeCallback,
            .GetBattery = GetBattery
        },
        .MotionDevice = motionDevice,
        .LastMovementTime = time_us_32(),
        .OnMotion = OnMotion,
        .Update = Update,
        .Screen = screen,
        .Module = module,
        .UpdateState = UpdateStateNormal,
        .Preferences = preferences
    };

    LinkedList_Init(&manager.SleepChangeCallbacks);

    memcpy(out, &manager, sizeof(manager));

    PicoBattery_Init(1000.0f, &out->Battery);
}