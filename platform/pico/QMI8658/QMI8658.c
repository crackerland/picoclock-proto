
//#include "stdafx.h"
#include "QMI8658.h"
#include <string.h>
#include "pico/sync.h"

#define QMI8658_Reset 0x60

#define IMU_INT1_GPIO_PIN 23
#define IMU_INT2_GPIO_PIN 24

// See 12.2 I2C Interface
// #define QMI8658_SLAVE_ADDR_L 0x6a
#define QMI8658_SLAVE_ADDR_H 0x6b // Alt device address. Enabled by external pull up on SA0.


// CTRL9 command completed.
#define QMI8658_STATUS1_CTRL9_CMD_DONE (0b1)

// WOM wake up.
#define QMI8658_STATUS1_WAKEUP_EVENT (0b100)

/* Transition times (7.2) */ 

// Time t0 - System turn on time: 1.75 seconds.
// * Transitioning from a no power state
// * Transitioning from a power down state
// * When a reset is issued
#define SYSTEM_TURN_ON_TIME_MILLIS 1750

// Time t1 
#define GYRO_WAKE_UP_TIME_MILLIS 60

// Time t2
#define ACCEL_WAKE_UP_TIME_MILLIS 3 

typedef struct PendingStatusEvents
{
    bool WomEvent;
    bool Ctrl9Event;
}
PendingStatusEvents;

typedef struct 
{
    DeferredTaskScheduler* Scheduler;
    DeferredTask* Task;
}
WomInterruptPayload;

typedef struct 
{
    Qmi8658* Module;
    void (*Callback)(void* payload);
    void* CallbackPayload;
}
WomTaskPayload;

// Serial interface and sensor enable. Register address: 2 (0x02)
typedef struct 
{
    // (SIM, Bit 7)
    // True: 3 wire SPI mode 
    // False: 4 wire SPI mode
    bool Spi3WireMode;

    // (SPI_AI) Bit 6
    // Whether serial addresses are autoincremented during a read.
    bool SerialAutoincrement;

    // (SPI_BE) Bit 5
    // Whether reads are returned as big endian.
    bool ReadBigEndian;

    // (SensorDisable) Bit 0
    // Whether the internal 2 MHz oscillator for the sensors is disabled.
    bool DisableSensors;
}
Ctrl1Values;

// Accelerometer settings. Register address 3 (0x03)
typedef struct  
{
    // (aST) Bit 7
    // Whether the accelerometer self test is enabled.
    bool SelfTest;

    // (aFS) Bits 6:4
    // The full scale value, i.e., the range.
    enum QMI8658_AccRange FullScale;

    // (aODR) Bits 3:0
    // The accelerometer output data rate.
    enum QMI8658_AccOdr OutputDataRate;
}
Ctrl2Values;

// Gyroscope settings. Register address 4 (0x04)
typedef struct 
{
    // (gST) Bit 7
    // Whether the gyroscope self test is enabled.
    bool SelfTest;

    // (gFS) Bits 6:4
    // The full scale value, i.e., the range.
    enum QMI8658_GyrRange FullScale;

    // (gODR) Bits 3:0
    // The gyroscope output data rate.
    enum QMI8658_GyrOdr OutputDataRate;
}
Ctrl3Values;

// Magnetometer settings: Register address 5 (0x05)
typedef struct
{
    // TODO: Not using the magnetometer, so not spending time on it for now.
    // (mDEV) Bits 6:3
    // (mODR) Bits 2:0
}
Ctrl4Values;

// Low pass filter bandwidth modes (Hz).
typedef enum 
{
    // Accelerometer: Low pass filter 2.62% of ODR.
    // Gyroscope: 2.62% of ODR.
    LowPassFilterMode0 = 0b00,

    // Accelerometer: Low pass filter 3.59% of ODR.
    // Gyroscope: Low pass filter 3.59% of ODR.
    LowPassFilterMode1 = 0b01,

    // Accelerometer: Low pass filter 5.32% of ODR.
    // Gyroscope: Low pass filter 5.32% of ODR.
    LowPassFilterMode2 = 0b10,

    // Accelerometer low pass filter 14.0% of ODR.
    // Gyroscope: Low pass filter 14.0% of ODR.
    LowPassFilterMode3 = 0b11
}
LowPassFilterMode;

// Sensor data processing settings. Register address: 6 (0x06)
typedef struct
{
    // (gLPF_MODE) Bits 6:5
    // The gyroscope low pass filter bandwidth.
    LowPassFilterMode GyroscopeLowPassFilterMode;

    // (gLPF_EN) Bit 4
    // Whether the gyroscope low pass filter is enabled.
    bool GyroscopeLowPassFilterEnabled;

    // (aLPF_MODE) Bits 2:1
    // The accelerometer low pass filter bandwidth.
    LowPassFilterMode AccelerometerLowPassFilterMode;

    // (aLPF_EN) Bit 0
    // Whether the accelerometer low pass filter is enabled.
    bool AccelerometerLowPassFilterEnabled;
}
Ctrl5Values;

// Attitude Engine ODR and motion on demand: Address: 7 (0x07)
typedef struct
{
    // (sMoD) Bit 7
    // Whether motion on demand is enabled. 
    // Requires Attitue Engine to be enabled (CTRL7 bit 3, "sEN") to enable motion on demand.
    bool MotionOnDemand;

    // (sODR) Bits 2:0
    // The output data rate for the Attitude Engine.
    enum QMI8658_AeOdr AttitudeEngineOutputDataRate;
}
Ctrl6Values;

// Enables sensors and configure data reads. Register address: 8 (0x08)
typedef struct 
{
    // (syncSmpl) Whether syncSmpl is enabled.
    bool SyncSmpl;

    // (sys_hs) Whether the high speed internal clock is used instead of a clock based on ODR.
    bool HighSpeedClock;

    // (gSN) Whether the gyroscope is in snooze mode (only drive enabled).
    // Requires gyroscope to be enabled (gEN).
    bool GyroscopeSnoozeMode;

    // (sEN) Whether Attitude Engine orientation and velocity increment computation is enabled.
    bool AttitudeEngineEnable;

    // (mEN) Whether the magnetometer is enabled.
    bool MagnetometerEnable;

    // (gEN) Whether the gyroscope is enabled.
    bool GyroscopeEnable;

    // (aEN) Whether the accelerometer is enabled.
    bool AccelerometerEnable;
}
Ctrl7Values;

typedef struct StatusIntValues
{
    // (Locked) Whether sensor data is locked.
    // If syncSmpl is enabled, this will have the same value as interrupt INT1.
    bool SensorDataLocked;

    // (Avail) Whether sensor data is available.
    // If syncSmpl is enabled, this will have the same value as interrupt INT2.
    bool SensorDataAvailable;
}
StatusIntValues;

typedef struct Ctrl9CalibrationRegisterData 
{
    enum QMI8658Register Register;
    uint8_t Value;
}
Ctrl9CalibrationRegisterData;

typedef struct Ctrl9CalibrationData  
{
    Ctrl9CalibrationRegisterData Registers[8];
    unsigned int RegisterCount;
}
Ctrl9CalibrationData;

typedef enum 
{
    I2cReadResult_Success,
    I2cReadResult_AddressWriteFailed,
    I2cReadResult_ReadPartial,
    I2cReadResult_ReadFailed
}
I2cReadResult;

static I2cReadResult ReadBytes(Qmi8658* module, uint8_t registerAddress, uint8_t* buffer, size_t registerReadCount)
{
    // First tell the module which register to read.
    int result = i2c_write_blocking(module->I2cInstance, module->ModuleSlaveAddress, &registerAddress, 1, true);
    if (!result || result == PICO_ERROR_GENERIC)
    {
        printf("I2C write failed - Register: %02X\n", registerAddress);
        return I2cReadResult_AddressWriteFailed;
    }

    // Then read the contents, starting at the first register until `registerReadCount`.
    // Since we enabled autoincrement, it will automatically move to each register.
    result = i2c_read_blocking(module->I2cInstance, module->ModuleSlaveAddress, buffer, registerReadCount, false);

    I2cReadResult ret = result == registerReadCount
        ? I2cReadResult_Success
        : result > 0 && result != PICO_ERROR_GENERIC
            ? I2cReadResult_ReadPartial
            : I2cReadResult_ReadFailed;

    if (ret != I2cReadResult_Success)
    {
        printf("I2C read failed\n");
    }

    return ret;

    // return result == registerReadCount
    //     ? I2cReadResult_Success
    //     : result > 0 && result != PICO_ERROR_GENERIC
    //         ? I2cReadResult_ReadPartial
    //         : I2cReadResult_ReadFailed;
}

static bool WriteSingle(Qmi8658* module, uint8_t registerAddress, uint8_t value)
{
    unsigned int retry = 0;
    int writeSucceeded = 0;
    while ((!writeSucceeded || writeSucceeded == PICO_ERROR_GENERIC) && retry++ < 5)
    {
        // First tell the module which register we're writing to, then follow it by the value.
        uint8_t data[2] = { registerAddress, value };
        writeSucceeded = i2c_write_blocking(module->I2cInstance, module->ModuleSlaveAddress, data, 2, false);
    }

    return writeSucceeded != PICO_ERROR_GENERIC;
}

// Starts writing at the given first register address using the first value, then iterates for the given length.
static unsigned int WriteMultiple(Qmi8658* module, uint8_t firstRegisterAddress, uint8_t *value, size_t length)
{
    unsigned int bytesWritten = 0;
    for (unsigned int i = 0; i < length; i++)
    {
        bytesWritten += WriteSingle(module, firstRegisterAddress++, value[i]);
    }

    return bytesWritten;
}

static inline unsigned int HertzToMillis(float hertz)
{
    return (unsigned int)1.0f / hertz * 1000;
}

static inline float GetGyroOdrHertz(enum QMI8658_GyrOdr odr)
{
    switch (odr)
    {
        case QMI8658GyrOdr_8000Hz:
            return 8000;

        case QMI8658GyrOdr_4000Hz:
            return 4000;

        case QMI8658GyrOdr_2000Hz:
            return 2000;

        case QMI8658GyrOdr_1000Hz:
            return 1000;

        case QMI8658GyrOdr_500Hz:
            return 500;

        case QMI8658GyrOdr_250Hz:
            return 250;

        case QMI8658GyrOdr_125Hz:
            return 125;

        case QMI8658GyrOdr_62_5Hz:
            return 62;

        case QMI8658GyrOdr_31_25Hz:
            return 31.25;
    }

    return 0;
}

static inline float GetAccelOdrHertz(enum QMI8658_AccOdr odr)
{
    switch (odr)
    {
        case QMI8658AccOdr_8000Hz:
            return 8000;

        case QMI8658AccOdr_4000Hz:
            return 4000;

        case QMI8658AccOdr_2000Hz:
            return 2000;

        case QMI8658AccOdr_1000Hz:
            return 1000;

        case QMI8658AccOdr_500Hz:
            return 500;

        case QMI8658AccOdr_250Hz:
            return 250;

        case QMI8658AccOdr_125Hz:
            return 125;

        case QMI8658AccOdr_62_5Hz:
            return 62.5;

        case QMI8658AccOdr_31_25Hz:
            return 31.25;

        case QMI8658AccOdr_LowPower_128Hz:
            return 128;

        case QMI8658AccOdr_LowPower_21Hz:
            return 21;

        case QMI8658AccOdr_LowPower_11Hz:
            return 11;

        case QMI8658AccOdr_LowPower_3Hz:
            return 3;
    }

    return 0;
}

// (Table 8) Gyro Turn On Time = 60 ms + 3/ODR 
// 
// (7.2) The Gyro Turn on Time (see Table 8) is comprised
// of t1 (the gyroscope wakeup time) and t5 (the part’s
// filter settling time). t1 is typically 60 ms and t5 is
// defined as 3/ODR, where ODR is the output data
// rate in Hertz.
static inline unsigned int GetGyroTurnOnTimeMicros(enum QMI8658_GyrOdr odr)
{ 
    return (unsigned int)(GYRO_WAKE_UP_TIME_MILLIS + (3.0f / HertzToMillis(GetGyroOdrHertz(odr)))) * 1000;
}

// The Accel Turn on Time (see Table 7) is comprised
// of t2 (the accelerometer wakeup time) and t5 (the
// part’s filter settling time). t2 is typically 3 ms, and t5
// is defined as 3/ODR, where ODR is the output data
// rate in Hertz
static inline unsigned int GetAccelTurnOnTimeMicros(enum QMI8658_AccOdr odr)
{ 
    return (unsigned int)(ACCEL_WAKE_UP_TIME_MILLIS + (3.0f / HertzToMillis(GetAccelOdrHertz(odr)))) * 1000;
}

// SIM 
#define CTRL1_SIM_MASK 0b10000000

// SPI_AI
#define CTRL1_SPI_AI_MASK 0b01000000

// SPI_BE
#define CTRL1_SPI_BE_MASK 0b00100000

// SensorDisable
#define CTRL1_SENSOR_DISABLE_MASK 0b00000001

static void ReadCtrl1(Qmi8658* module, Ctrl1Values* out)
{
    uint8_t value = 0;
    ReadBytes(module, QMI8658Register_Ctrl1, &value, 1);

    Ctrl1Values values = 
    {
        .Spi3WireMode = !!(value & CTRL1_SIM_MASK),
        .SerialAutoincrement = !!(value & CTRL1_SPI_AI_MASK),
        .ReadBigEndian = !!(value & CTRL1_SPI_BE_MASK),
        .DisableSensors = !!(value & CTRL1_SENSOR_DISABLE_MASK)
    };

    memcpy(out, &values, sizeof(values));
}

static void ConfigureCtrl1(Qmi8658* module, Ctrl1Values* config)
{
    WriteSingle(
        module, 
        QMI8658Register_Ctrl1, 
        (config->Spi3WireMode ? CTRL1_SIM_MASK : 0)
            | (config->SerialAutoincrement ? CTRL1_SPI_AI_MASK : 0)
            | (config->ReadBigEndian ? CTRL1_SPI_BE_MASK : 0)
            | (config->DisableSensors ? CTRL1_SENSOR_DISABLE_MASK : 0));
}

// aST
#define CTRL2_ACC_SELF_TEST_MASK 0b10000000

// aFS
#define CTRL2_ACC_FULL_SCALE_MASK 0b01110000
#define CTRL2_ACC_FULL_SCALE_LSB 4

// aODR
#define CTRL2_ACC_ODR_MASK 0b00001111

static void ConfigureCtrl2(Qmi8658* module, Ctrl2Values* config)
{
    WriteSingle(
        module, 
        QMI8658Register_Ctrl2, 
        (config->SelfTest ? CTRL2_ACC_SELF_TEST_MASK : 0)
            | config->FullScale
            | config->OutputDataRate);
}

static void ReadCtrl2(Qmi8658* module, Ctrl2Values* out)
{    
    uint8_t value = 0;
    ReadBytes(module, QMI8658Register_Ctrl2, &value, 1);

    Ctrl2Values values = 
    {
        .SelfTest = !!(value & CTRL2_ACC_SELF_TEST_MASK),
        .FullScale = (value & CTRL2_ACC_FULL_SCALE_MASK) >> CTRL2_ACC_FULL_SCALE_LSB,
        .OutputDataRate = value & CTRL2_ACC_ODR_MASK
    };

    memcpy(out, &values, sizeof(values));
}

// gST
#define CTRL3_GYRO_SELF_TEST_MASK 0b10000000

// gFS
#define CTRL3_GYRO_FULL_SCALE_MASK 0b01110000
#define CTRL3_GYRO_FULL_SCALE_LSB 4

// gODR
#define CTRL3_GYRO_ODR_MASK 0b00001111

static void ConfigureCtrl3(Qmi8658* module, Ctrl3Values* config)
{
    WriteSingle(
        module, 
        QMI8658Register_Ctrl3, 
        (config->SelfTest ? CTRL3_GYRO_SELF_TEST_MASK : 0)
            | config->FullScale
            | config->OutputDataRate);
}

static void ReadCtrl3(Qmi8658* module, Ctrl3Values* out)
{    
    uint8_t value = 0;
    ReadBytes(module, QMI8658Register_Ctrl3, &value, 1);

    Ctrl3Values values = 
    {
        .SelfTest = !!(value & CTRL3_GYRO_SELF_TEST_MASK),
        .FullScale = (value & CTRL3_GYRO_FULL_SCALE_MASK) >> CTRL3_GYRO_FULL_SCALE_LSB,
        .OutputDataRate = value & CTRL3_GYRO_ODR_MASK
    };

    memcpy(out, &values, sizeof(values));
}

// gLPF_MODE
#define CTRL5_GYRO_LPF_MODE_MASK 0b01100000
#define CTRL5_GYRO_LPF_MODE_LSB 5

// gLPF_EN
#define CTRL5_GYRO_LPF_ENABLE_MASK 0b00010000

// aLPF_MODE
#define CTRL5_ACC_LPF_MODE_MASK 0b00000110
#define CTRL5_ACC_LPF_MODE_LSB 1

// aLPF_EN
#define CTRL5_ACC_LPF_ENABLE_MASK 0b00000001

static void ConfigureCtrl5(Qmi8658* module, Ctrl5Values* config)
{
    WriteSingle(
        module, 
        QMI8658Register_Ctrl5, 
        (config->GyroscopeLowPassFilterMode << CTRL5_GYRO_LPF_MODE_LSB)
            | (config->GyroscopeLowPassFilterEnabled ? CTRL5_GYRO_LPF_ENABLE_MASK : 0)
            | (config->AccelerometerLowPassFilterMode << CTRL5_ACC_LPF_MODE_LSB)
            | (config->AccelerometerLowPassFilterEnabled ? CTRL5_ACC_LPF_ENABLE_MASK : 0));
}

static void ReadCtrl5(Qmi8658* module, Ctrl5Values* out)
{    
    uint8_t value = 0;
    ReadBytes(module, QMI8658Register_Ctrl5, &value, 1);

    Ctrl5Values values = 
    {
        .GyroscopeLowPassFilterMode = (value & CTRL5_GYRO_LPF_MODE_MASK) >> CTRL5_GYRO_LPF_MODE_LSB,
        .GyroscopeLowPassFilterEnabled = !!(value & CTRL5_GYRO_LPF_ENABLE_MASK),
        .AccelerometerLowPassFilterMode = (value & CTRL5_ACC_LPF_MODE_MASK) >> CTRL5_ACC_LPF_MODE_LSB, 
        .AccelerometerLowPassFilterEnabled = !!(value & CTRL5_ACC_LPF_ENABLE_MASK)
    };

    memcpy(out, &values, sizeof(values));
}

// sMoD
#define CTRL6_MOD_ENABLE_MASK 0b10000000

// sODR
#define CTRL6_AE_ODR_MASK 0b00000111

static void ConfigureCtrl6(Qmi8658* module, Ctrl6Values* config)
{    
    WriteSingle(
        module, 
        QMI8658Register_Ctrl6, 
        (config->MotionOnDemand ? CTRL6_MOD_ENABLE_MASK : 0)
            | config->AttitudeEngineOutputDataRate);
}

static void ReadCtrl6(Qmi8658* module, Ctrl6Values* out)
{    
    uint8_t value = 0;
    ReadBytes(module, QMI8658Register_Ctrl6, &value, 1);

    Ctrl6Values values = 
    {
        .MotionOnDemand = !!(value & CTRL6_MOD_ENABLE_MASK),
        .AttitudeEngineOutputDataRate = value & CTRL6_AE_ODR_MASK,
    };

    memcpy(out, &values, sizeof(values));
}

// syncSmpl
#define CTRL7_SYNC_SMPL_MASK 0b10000000

// sys_hs
#define CTRL7_SYS_HS_MASK 0b01000000

// gSN
#define CTRL7_G_SN_MASK 0b00010000

// sEN
#define CTRL7_S_EN_MASK 0b00001000

// mEN
#define CTRL7_M_EN_MASK 0b00000100

// gEN
#define CTRL7_G_EN_MASK 0b00000010

// aEN
#define CTRL7_A_EN_MASK 0b00000001

static void ConfigureCtrl7(Qmi8658* module, Ctrl7Values* config)
{
    WriteSingle(
        module, 
        QMI8658Register_Ctrl7, 
        (config->SyncSmpl ? CTRL7_SYNC_SMPL_MASK : 0)
            | (config->HighSpeedClock ? CTRL7_SYS_HS_MASK : 0)
            | (config->GyroscopeSnoozeMode ? CTRL7_G_SN_MASK : 0)
            | (config->AttitudeEngineEnable ? CTRL7_S_EN_MASK : 0)
            | (config->MagnetometerEnable ? CTRL7_M_EN_MASK : 0)
            | (config->GyroscopeEnable ? CTRL7_G_EN_MASK : 0)
            | (config->AccelerometerEnable ? CTRL7_A_EN_MASK : 0));
}

static void ReadCtrl7(Qmi8658* module, Ctrl7Values* out)
{    
    uint8_t value = 0;
    ReadBytes(module, QMI8658Register_Ctrl7, &value, 1);

    Ctrl7Values values = 
    {
        .SyncSmpl = !!(value & CTRL7_SYNC_SMPL_MASK),
        .HighSpeedClock = !!(value & CTRL7_SYS_HS_MASK),
        .GyroscopeSnoozeMode = !!(value & CTRL7_G_SN_MASK),
        .AttitudeEngineEnable = !!(value & CTRL7_S_EN_MASK),
        .MagnetometerEnable = !!(value & CTRL7_M_EN_MASK),
        .GyroscopeEnable = !!(value & CTRL7_G_EN_MASK),
        .AccelerometerEnable = !!(value & CTRL7_A_EN_MASK),
    };

    memcpy(out, &values, sizeof(values));
}

static void DumpConfig(Qmi8658* module)
{
    Ctrl1Values ctrl1;
    ReadCtrl1(module, &ctrl1);

    Ctrl2Values ctrl2;
    ReadCtrl2(module, &ctrl2);

    Ctrl3Values ctrl3;
    ReadCtrl3(module, &ctrl3);

    Ctrl5Values ctrl5;
    ReadCtrl5(module, &ctrl5);

    Ctrl6Values ctrl6;
    ReadCtrl6(module, &ctrl6);

    Ctrl7Values ctrl7;
    ReadCtrl7(module, &ctrl7);

    printf("CTRL1: \n");
    printf("SIM: %d; SensorDisable: %d; SPI_BE: %d; SPI_AI: %d\n", ctrl1.Spi3WireMode, ctrl1.DisableSensors, ctrl1.ReadBigEndian, ctrl1.SerialAutoincrement);

    printf("CTRL2: \n");
    printf("aFS: %d; aODR: %d; aST: %d\n", ctrl2.FullScale, ctrl2.OutputDataRate, ctrl2.SelfTest);

    printf("CTRL3: \n");
    printf("gFS: %d; gODR: %d; gST: %d\n", ctrl3.FullScale, ctrl3.OutputDataRate, ctrl3.SelfTest);

    printf("CTRL5: \n");
    printf("aLPF_EN: %d; aLPF_MODE: %d; gLPF_EN: %d; gLPF_MODE: %d\n", ctrl5.AccelerometerLowPassFilterEnabled, ctrl5.AccelerometerLowPassFilterMode, ctrl5.GyroscopeLowPassFilterEnabled, ctrl5.GyroscopeLowPassFilterMode);

    printf("CTRL6: \n");
    printf("sMoD: %d; sODR: %d\n", ctrl6.MotionOnDemand, ctrl6.AttitudeEngineOutputDataRate);

    printf("CTRL7: \n");
    printf("syncSmpl: %d; sys_hs: %d; gSN: %d; sEN: %d; mEN: %d; gEN: %d; aEN: %d\n", ctrl7.SyncSmpl, ctrl7.HighSpeedClock, ctrl7.GyroscopeSnoozeMode, ctrl7.AttitudeEngineEnable, ctrl7.MagnetometerEnable, ctrl7.GyroscopeEnable, ctrl7.AccelerometerEnable);
}

// Locked
#define STATUSINT_LOCKED_MASK 0b00000010

// Avail
#define STATUSINT_AVAIL_MASK 0b00000001

static void ReadStatusInt(Qmi8658* module, StatusIntValues* out)
{    
    uint8_t value = 0;
    ReadBytes(module, QMI8658Register_StatusInt, &value, 1);

    StatusIntValues values = 
    {
        .SensorDataLocked = !!(value & STATUSINT_LOCKED_MASK),
        .SensorDataAvailable = !!(value & STATUSINT_AVAIL_MASK),
    };

    memcpy(out, &values, sizeof(StatusIntValues));
}

// sDA
#define STATUS0_S_DA_MASK 0b00001000

// mDA
#define STATUS0_M_DA_MASK 0b00000100

// gDA
#define STATUS0_G_DA_MASK 0b00000010

// aDA
#define STATUS0_A_DA_MASK 0b00000001

static void ReadStatus0(Qmi8658* module, Status0Values* out)
{    
    uint8_t value = 0;
    ReadBytes(module, QMI8658Register_Status0, &value, 1);

    Status0Values values = 
    {
        .AttitudeEngineDataAvailable = !!(value & STATUS0_S_DA_MASK),
        .MagnetometerDataAvailable = !!(value & STATUS0_M_DA_MASK),
        .GyroscopeDataAvailable = !!(value & STATUS0_G_DA_MASK),
        .AccelerometerDataAvailable = (value & STATUS0_A_DA_MASK)
    };

    memcpy(out, &values, sizeof(Status0Values));
}

// CmdDone
#define STATUS1_CMD_DONE_MASK 0b00000001

// (reserved/WoM - See 9.3) Wake on motion event
#define STATUS1_WOM_EVENT_MASK 0b00000100

// NOTE: Status1 values do not get set for some reason. 
// Not clear if it's a bug or a hardware malfunction or 
// bad documentation. It's better to just rely on the 
// interrupt.
static bool ReadStatus1(Qmi8658* module, Status1Values* out)
{
    volatile uint8_t value = 0;
    if (ReadBytes(module, QMI8658Register_Status1, (uint8_t*)&value, 1) != I2cReadResult_Success)
    {
        return false;
    }

    Status1Values values = 
    {
        .WakeOnMotionEvent = !!(value & STATUS1_WOM_EVENT_MASK),
        .Ctrl9CommandDone = !!(value & STATUS1_CMD_DONE_MASK),
    };

    memcpy(out, &values, sizeof(Status1Values));
    return true;
}

static volatile bool int1Pending = false;
static volatile bool int2Pending = false;
static InterruptCallback* int1Callback;
static InterruptCallback* int2Callback;
// static PendingStatusEvents pendingStatus = { };

static InterruptCallback* CreateInterruptCallback(
    Qmi8658* module, 
    InterruptCallbackHandler onInterruptReceived, 
    void* payload,
    void (*dispose)(InterruptCallback*))
{
    InterruptCallback callback = 
    {
        .OnInterruptReceived = onInterruptReceived,
        .Payload = payload,
        .Module = module,
        .Dispose = dispose 
    }; 

    InterruptCallback* out = (InterruptCallback*)malloc(sizeof(InterruptCallback));
    memcpy(out, &callback, sizeof(InterruptCallback));
    return out;
}

static void AddInterruptCallback(InterruptCallback* callback, InterruptCallback** first)
{
    InterruptCallback* next = *first;
    if (!next)
    {
        *first = callback;
    }
    else
    {
        while (next->Next)
        {
            next = next->Next;
        }

        next->Next = callback;
    }
}

static void RemoveInterruptCallback(InterruptCallback* callback, InterruptCallback** first)
{
    if (*first == callback)
    {
        *first = callback->Next; 
    }
    else
    {
        InterruptCallback* next = *first;
        while (next)
        {
            if (next->Next == callback)
            {
                next->Next = callback->Next;
                break;
            }
        }
    }
}

static inline void NotifyInterruptCallbacks(InterruptCallback* first)
{
    InterruptCallback* next = first;
    while (next)
    {
        InterruptCallback* nextNext = next->Next;
        (*next->OnInterruptReceived)(next);
        next = nextNext;
    }
}

static inline void AwaitInt1()
{
    while (!int1Pending)
    {
        tight_loop_contents();
    }

    int1Pending = false;
}

static inline void AwaitInt2()
{
    while (!int2Pending)
    {
        tight_loop_contents();
    }

    int2Pending = false;
}

static void PollInt1(Qmi8658* module)
{
    AwaitInt1();
    NotifyInterruptCallbacks(module->Interrupt1Callbacks);
}

static void PollInt2(Qmi8658* module)
{
    AwaitInt2();
    NotifyInterruptCallbacks(module->Interrupt2Callbacks);
}

static void OnGpioInterruptReceived(uint gpio, uint32_t eventMask)
{
    if (gpio == IMU_INT1_GPIO_PIN)
    {
        int1Pending = true;
        NotifyInterruptCallbacks(int1Callback);
    }
    else if (gpio == IMU_INT2_GPIO_PIN) 
    {
        int2Pending = true;
        NotifyInterruptCallbacks(int2Callback);
    }
}

// static void ReadInterruptStatus1(InterruptCallback* callback)
// {
//     Status1Values status1;
//     ReadStatus1(callback->Module, &status1);

//     if (status1.Ctrl9CommandDone)
//     {
//         pendingStatus.Ctrl9Event = true;
//     }

//     if (status1.WakeOnMotionEvent)
//     {
//         pendingStatus.WomEvent = true;
//     }
// }

static void OnCtrl9CommandExecuted(InterruptCallback* callback)
{
    // StatusIntValues statusInt;
    // ReadStatusInt(callback->Module, &statusInt);

    // Status1Values status1;
    // ReadStatus1(callback->Module, &status1);
    // if (!ReadStatus1(callback->Module, &status1) || !status1.Ctrl9CommandDone)
    // {
    //     return;
    // }

    // if (!pendingStatus.Ctrl9Event)
    // {
    //     return;
    // }

    // pendingStatus.Ctrl9Event = false;

    *((bool*)callback->Payload) = false;
    RemoveInterruptCallback(callback, &callback->Module->Interrupt1Callbacks);
}

static void RunCtrl9ReadCommand(Qmi8658* module, enum QMI8658_Ctrl9Command command)
{
    // volatile bool waiting = true;
    // AddInterruptCallback( 
    //     CreateInterruptCallback(module, OnCtrl9CommandExecuted, (void*)&waiting),
    //     &module->Interrupt1Callbacks);

    WriteSingle(module, QMI8658Register_Ctrl9, command);

    // HACK: Status1 not being read correctly. INT dependency not working.
    sleep_ms(100);

    // while (waiting)
    // {
    //     PollInt1(module);
    // }
}

static void RunCtrl9WriteCommand(Qmi8658* module, enum QMI8658_Ctrl9Command command, Ctrl9CalibrationData* registers)
{
    for (unsigned int i = 0; i < registers->RegisterCount; i++)
    {
        Ctrl9CalibrationRegisterData* data = &registers->Registers[i];
        WriteSingle(module, data->Register, data->Value);
    }

    RunCtrl9ReadCommand(module, command);
}

static inline float ReadAccAxis(MotionDevice* device, short lo, short hi)
{
    return (float)(((short)((unsigned short)(hi << 8) | lo)) * 1000.0f) / device->VectorLsbDivider;
}

static inline float ReadGyroAxis(MotionDevice* device, short lo, short hi)
{
    return (float)(((short)((unsigned short)(hi << 8) | lo)) * 1.0f) / device->VectorLsbDivider;
}

static void ReadAccelerometer(MotionDevice* device, Qmi8658_MotionVector* vectorOut, Qmi8658_Quaternion* quaternionOut)
{
    unsigned char registersBuffer[6] = { };
    ReadBytes(device->Module, QMI8658Register_Ax_L, registersBuffer, 6);
    vectorOut->X = ReadAccAxis(device, registersBuffer[0], registersBuffer[1]);
    vectorOut->Y = ReadAccAxis(device, registersBuffer[2], registersBuffer[3]);
    vectorOut->Z = ReadAccAxis(device, registersBuffer[4], registersBuffer[5]);
    memset(quaternionOut, 0, sizeof(Qmi8658_Quaternion));
}

static void ReadGyro(MotionDevice* device, Qmi8658_MotionVector* vectorOut, Qmi8658_Quaternion* quaternionOut)
{
    uint8_t registersBuffer[6] = { };
    ReadBytes(device->Module, QMI8658Register_Gx_L, registersBuffer, 6);
    vectorOut->X = ReadGyroAxis(device, registersBuffer[0], registersBuffer[1]);
    vectorOut->Y = ReadGyroAxis(device, registersBuffer[2], registersBuffer[3]);
    vectorOut->Z = ReadGyroAxis(device, registersBuffer[4], registersBuffer[5]);
    memset(quaternionOut, 0, sizeof(Qmi8658_Quaternion));
}

static void ReadAttitudeEngine(MotionDevice* device, Qmi8658_MotionVector* velocityOut, Qmi8658_Quaternion* quaternionOut)
{
    uint8_t registersBuffer[14] = { };
    ReadBytes(device->Module, QMI8658Register_Q1_L, registersBuffer, sizeof(registersBuffer));

    int16_t raw_q_xyz[4] = 
    {
        (int16_t)((registersBuffer[1] << 8) | registersBuffer[0]),
        (int16_t)((registersBuffer[3] << 8) | registersBuffer[2]),
        (int16_t)((registersBuffer[5] << 8) | registersBuffer[4]),
        (int16_t)((registersBuffer[7] << 8) | registersBuffer[6])
    };

    int16_t raw_v_xyz[3] =
    {
        (int16_t)((registersBuffer[9] << 8) | registersBuffer[8]),
        (int16_t)((registersBuffer[11] << 8) | registersBuffer[10]),
        (int16_t)((registersBuffer[13] << 8) | registersBuffer[12])
    };

    quaternionOut->W = (float)(raw_q_xyz[0] * 1.0f) / device->QuaternionLsbDivider;
    quaternionOut->X = (float)(raw_q_xyz[1] * 1.0f) / device->QuaternionLsbDivider;
    quaternionOut->Y = (float)(raw_q_xyz[2] * 1.0f) / device->QuaternionLsbDivider;
    quaternionOut->Z = (float)(raw_q_xyz[3] * 1.0f) / device->QuaternionLsbDivider;

    velocityOut->X = (float)(raw_v_xyz[0] * 1.0f) / device->VectorLsbDivider;
    velocityOut->Y = (float)(raw_v_xyz[1] * 1.0f) / device->VectorLsbDivider;
    velocityOut->Z = (float)(raw_v_xyz[2] * 1.0f) / device->VectorLsbDivider;
}

static void DisposeAeDataEvent(InterruptCallback* callback)
{
    RemoveInterruptCallback(
        callback,
        &callback->Module->Interrupt2Callbacks);
}

static void OnAeDataAvailable(InterruptCallback* callback)
// static void OnAeDataAvailable(InterruptCallback* callback, Status0Values* status0, Status1Values* status1)
{
    // if (!status0->AttitudeEngineDataAvailable)
    // {
    //     return;
    // }
    Status0Values status0;
    ReadStatus0(callback->Module, &status0);
    if (!status0.AttitudeEngineDataAvailable)
    {
        return;
    }

    *((bool*)callback->Payload) = false;
    DisposeAeDataEvent(callback);
}

static inline Task* QueryMotionOnDemand(Qmi8658* module) 
{
    // (5.8.5 CTRL9 Commands in Detail) [Errata from copied QMI8610 datasheet]
    // ...
    // The CTRL_CMD_REQ_MoD command is then issued by
    // writing 0x0C [Correction: 0x03] to CTRL9 register 0x0A. This indicates to
    // the QMI8658C that it is required to supply the motion data
    // to the host. The device immediately makes available the
    // orientation and velocity increments it has computed so far
    // to the host by making them available at output registers
    // 0x25 to 0x3D [Correction: 0x41-0x56] and raises the INT1 to indicate to the host
    // that valid data is available.
    bool waiting = true;

    // (6.1.2: AE Mode) In AE Mode, INT2 is asserted when data is available.
    AddInterruptCallback( 
        CreateInterruptCallback(module, OnAeDataAvailable, &waiting, DisposeAeDataEvent),
        &module->Interrupt2Callbacks);

    WriteSingle(module, QMI8658Register_Ctrl9, QMI8658_Ctrl9_Cmd_Rqst_Sdi_Mod);
    // RunCtrl9ReadCommand(module, QMI8658_Ctrl9_Cmd_Rqst_Sdi_Mod);

    while (waiting)
    {
        PollInt2(module);
    }
}

static void ReadAttitudeEngineOnDemand(MotionDevice* device, Qmi8658_MotionVector* velocityOut, Qmi8658_Quaternion* quaternionOut)
{
    QueryMotionOnDemand(device->Module);
    ReadAttitudeEngine(device, velocityOut, quaternionOut);
}

// Returns the LSB divider based on data rate.
static inline unsigned short ConfigureAccelerometer(Qmi8658* module, Qmi8658AccelerometerConfig* config)
{
    ConfigureCtrl2(
        module, 
        &(Ctrl2Values)
        {
            .SelfTest = config->SelfTestEnabled,
            .FullScale = config->Range,
            .OutputDataRate = config->Odr
        });

    Ctrl5Values ctrl5Config;
    ReadCtrl5(module, &ctrl5Config);

    if (config->LowPassFilterEnabled)
    {
        ctrl5Config.AccelerometerLowPassFilterMode = LowPassFilterMode3;
        ctrl5Config.AccelerometerLowPassFilterEnabled = true;
    }
    else
    {
        ctrl5Config.AccelerometerLowPassFilterEnabled = false;
    }

    ConfigureCtrl5(module, &ctrl5Config);

    // Return the LSB divider.
    switch (config->Range)
    {
        case QMI8658AccRange_2g:
            return (1 << 14);

        case QMI8658AccRange_4g:
            return (1 << 13);

        case QMI8658AccRange_8g:
            return (1 << 12);

        case QMI8658AccRange_16g:
            return (1 << 11);

        default:
            config->Range = QMI8658AccRange_8g;
            return (1 << 12);
    }
}

// Returns the LSB divider based on data rate.
static inline unsigned short ConfigureGyroscope(Qmi8658* module, Qmi8658GyroscopeConfig* config)
{
    ConfigureCtrl3(
        module,
        &(Ctrl3Values)
        {
            .SelfTest = config->SelfTestEnabled,
            .FullScale = config->Range,
            .OutputDataRate = config->Odr
        });

    Ctrl5Values ctrl5Config;
    ReadCtrl5(module, &ctrl5Config);

    if (config->LowPassFilterEnabled)
    {
        ctrl5Config.GyroscopeLowPassFilterMode = LowPassFilterMode3;
        ctrl5Config.GyroscopeLowPassFilterEnabled = true;
    }
    else
    {
        ctrl5Config.GyroscopeLowPassFilterEnabled = false;
    }

    ConfigureCtrl5(module, &ctrl5Config);

    // Return the LSB divider.
    switch (config->Range)
    {
        case QMI8658GyrRange_16dps:
            return 2048;
            
        case QMI8658GyrRange_32dps:
            return 1024;
            
        case QMI8658GyrRange_64dps:
            return 512;
            
        case QMI8658GyrRange_128dps:
            return 256;
            
        case QMI8658GyrRange_256dps:
            return 128;
            
        case QMI8658GyrRange_512dps:
            return 64;
            
        case QMI8658GyrRange_1024dps:
            return 32;

        case QMI8658GyrRange_2048dps:
            return 16;
            
        // case QMI8658GyrRange_4096dps:
        //     module->Gyroscope.LsbDivider = 8;
        //     break;

        default:
            config->Range = QMI8658GyrRange_512dps;
            return 64;
    }
}

static void ConfigureAttitudeEngine(
    Qmi8658* module,
    bool enableMotionOnDemand,
    enum QMI8658_AeOdr dataRate,
    MotionDevice* attitudeEngineOut)
{
    memcpy(
        &module->AccelConfig, 
        &(Qmi8658AccelerometerConfig)
        {
            .Enabled = true,
            .Odr = QMI8658AccOdr_1000Hz,
            .Range = QMI8658AccRange_8g
        },
        sizeof(Qmi8658AccelerometerConfig));

    ConfigureCtrl2(
        module,
        // &(Ctrl2Values) { .FullScale = QMI8658AccRange_8g });
        &(Ctrl2Values) { .FullScale = QMI8658AccRange_8g, .OutputDataRate = QMI8658AccOdr_1000Hz });
        // &(Ctrl2Values) { .FullScale = QMI8658AccRange_8g, .OutputDataRate = QMI8658AccOdr_LowPower_128Hz });
        // &(Ctrl2Values) { .FullScale = QMI8658AccRange_8g, .OutputDataRate = QMI8658AccOdr_8000Hz });

    ConfigureCtrl3(
        module,
        // &(Ctrl3Values) { .FullScale = QMI8658GyrRange_512dps });
        &(Ctrl3Values) { .FullScale = QMI8658GyrRange_512dps, .OutputDataRate = QMI8658GyrOdr_1000Hz });

    ConfigureCtrl5(
        module, 
        &(Ctrl5Values) 
        { 
            // .AccelerometerLowPassFilterEnabled = true, 
            // .AccelerometerLowPassFilterMode = LowPassFilterMode3,
            // .GyroscopeLowPassFilterEnabled = true,
            // .GyroscopeLowPassFilterMode = LowPassFilterMode3
        });

    ConfigureCtrl6(
        module,
        &(Ctrl6Values)
        {
            .AttitudeEngineOutputDataRate = dataRate,
            .MotionOnDemand = enableMotionOnDemand
        });

    ConfigureCtrl7(
        module,
        &(Ctrl7Values)
        {
            .SyncSmpl = true,
            .AttitudeEngineEnable = true,
            .AccelerometerEnable = true,
            .GyroscopeEnable = true
        });

    sleep_us(GetAccelTurnOnTimeMicros(QMI8658AccOdr_1000Hz) + GetGyroTurnOnTimeMicros(QMI8658GyrOdr_1000Hz));

    memcpy(
        attitudeEngineOut,
        &(MotionDevice)
        {
            .Read = enableMotionOnDemand ? ReadAttitudeEngineOnDemand : ReadAttitudeEngine,
            .Module = module,
            .VectorLsbDivider = 1 << 10,
            .QuaternionLsbDivider = 1 << 14
        },
        sizeof(MotionDevice));
}

static void ConfigureSensors(
    Qmi8658* module,
    Qmi8658AccelerometerConfig* accelConfig, 
    Qmi8658GyroscopeConfig* gyroConfig,
    MotionDevice* accelerometerOut,
    MotionDevice* gyroscopeOut)
{
    unsigned int startupTimeUs = 0;
    if (accelConfig->Enabled)
    {
        startupTimeUs += GetAccelTurnOnTimeMicros(accelConfig->Odr);

        memcpy(
            accelerometerOut, 
            &(MotionDevice)
            {
                .Read = ReadAccelerometer,
                .Module = module,
                .VectorLsbDivider = ConfigureAccelerometer(module, accelConfig)
            }, 
            sizeof(MotionDevice));
    }

    if (gyroConfig->Enabled)
    {
        startupTimeUs += GetGyroTurnOnTimeMicros(gyroConfig->Odr);

        memcpy(
            gyroscopeOut, 
            &(MotionDevice)
            {
                .Read = ReadGyro,
                .Module = module,
                .VectorLsbDivider = ConfigureGyroscope(module, gyroConfig)
            }, 
            sizeof(MotionDevice));
    }

    // Enable the selected sensors.
    ConfigureCtrl7(
        module, 
        &(Ctrl7Values)
        { 
            // .SyncSmpl = true,
            .AttitudeEngineEnable = false,
            .AccelerometerEnable = accelConfig->Enabled,
            .GyroscopeEnable = gyroConfig->Enabled 
        });

    sleep_us(startupTimeUs);

    module->AttitudeEngineEnabled = false;
    memcpy(&module->AccelConfig, accelConfig, sizeof(Qmi8658AccelerometerConfig));
    memcpy(&module->GyroConfig, gyroConfig, sizeof(Qmi8658GyroscopeConfig));

    DumpConfig(module);
}

static void WomTask(DeferredTask* task, void* payload, void* output)
{
    WomTaskPayload* womPayload = (WomTaskPayload*)payload;
    if (womPayload->Callback)
    {
        (*womPayload->Callback)(womPayload->CallbackPayload);
    }
    
    free(payload);
}

static void NoOpInt(InterruptCallback* callback)
{
}

static void DisposeWomEvent(InterruptCallback* callback)
{
    WomInterruptPayload* payload = (WomInterruptPayload*)callback->Payload;
    RemoveInterruptCallback(callback, &int1Callback);
    free(payload);
}

static void OnWomEvent(InterruptCallback* callback)
{
    // if (!pendingStatus.WomEvent)
    // {
    //     return;
    // }

    // pendingStatus.WomEvent = false;

    callback->OnInterruptReceived = NoOpInt;
    WomInterruptPayload* payload = (WomInterruptPayload*)callback->Payload;
    (*payload->Scheduler->Schedule)(payload->Scheduler, payload->Task);
    DisposeWomEvent(callback);
}

static InterruptCallback* EnableWakeOnMotion(Qmi8658* module, void (*onWake)(void* payload), void* callbackPayload)
{
    module->WakeOnMotionEnabled = true;

    // QMI8658C doc 9.4 "Configuration Procedure":
    // The host processor is responsible for all configurations
    // necessary to put the QMI8658C into WoM mode. The
    // specific sequence of operations performed by the host
    // processor to enable WoM is shown in Figure 10
    // (Figure 10)
    // 1. [Disable sensors. (Write 0x00 to CTRL7)]
    // 2. [Set Accelerometer sample rate and scale (Write CTRL2)]
    // 3. [Set Wake on Motion (WoM) Threshold in CAL1_L; select interrupt, polarity and blanking time in CAL1_H]
    // 4. [Execute CTRL9 command to configure WoM mode]
    // 5. [Set Accelerometer enable bit in CTRL7]

    // 1. Disable sensors.
    ConfigureCtrl7(module, &(Ctrl7Values) { });

    // 2. Set accelerometer sample rate and scale.
    Qmi8658AccelerometerConfig accelConfig = 
    {
        .Enabled = true,
        .Odr = QMI8658AccOdr_LowPower_3Hz, // Spec sheet only defines aODR = 15 in Table 8, "Operating Mode Transition Diagram"
        // .Odr = QMI8658AccOdr_LowPower_21Hz, // it clearly works with 21 Hz too though.
        .Range = QMI8658AccRange_2g,
        .LowPassFilterEnabled = false,
        .SelfTestEnabled = false
    };

    ConfigureAccelerometer(module, &accelConfig);

    // 3. Set wake on motion threshold (WoM Threshold: absolute value in mg (with 1mg/LSB resolution)).
    // Select interrupt, polarity, and blanking time.
    // 4. Enable WOM through the matching CTRL9 command.
    // CAL1_H register:
    // 7:6 Interrupt select
    // 0:5 Interrupt blanking time (in number of accelerometer samples)
    const unsigned char blankingTimeMask = 0b00111111;
    Ctrl9CalibrationData cal = 
    {
        .Registers = 
        {
            // WoM Threshold: absolute value in mg (with 1mg/LSB resolution).
            (Ctrl9CalibrationRegisterData) 
            { 
                .Register = QMI8658Register_Cal1_L,
                .Value = (uint8_t)QMI8658WomThreshold_high
            },
            (Ctrl9CalibrationRegisterData) 
            { 
                .Register = QMI8658Register_Cal1_H,

                // Setting interrupt select to 00 - INT1 with initial value 0.
                // Setting blanking time to 4 samples.
                .Value = (uint8_t)QMI8658_Int1 | (uint8_t)QMI8658State_low | (0x04 & blankingTimeMask)
            },
        },
        .RegisterCount = 2
    };

    RunCtrl9WriteCommand(module, QMI8658_Ctrl9_Cmd_WoM_Setting, &cal);
    
    ConfigureCtrl7(module, &(Ctrl7Values) { .AccelerometerEnable = true });
    sleep_us(GetAccelTurnOnTimeMicros(accelConfig.Odr));

    WomTaskPayload* taskPayload = (WomTaskPayload*)malloc(sizeof(WomTaskPayload));
    taskPayload->Module = module;
    taskPayload->Callback = onWake;
    taskPayload->CallbackPayload = callbackPayload;
    DeferredTask* womTask = (DeferredTask*)malloc(sizeof(DeferredTask));
    DeferredTask_Init(WomTask, taskPayload, womTask);

    WomInterruptPayload* intPayload = (WomInterruptPayload*)malloc(sizeof(WomInterruptPayload));
    intPayload->Scheduler = module->Scheduler;
    intPayload->Task = womTask;

    InterruptCallback* callback = CreateInterruptCallback(module, OnWomEvent, intPayload, DisposeWomEvent);
    AddInterruptCallback(callback, &int1Callback);
    return callback;
}

static void DisableWakeOnMotion(Qmi8658* module)
{
    module->WakeOnMotionEnabled = false;

    // 9.6 Exiting Wake on Motion Mode
    // To exit WoM mode the host processor must first clear
    // CTRL7 to disable all sensors, and then write a threshold
    // value of 0x0 for the WoM Threshold (see Table 33,
    // Registers used for WoM) and execute the WoM
    // configuration CTRL9 command (see write-up for
    // CTRL_CMD_WRITE_WOM_SETTING in Section 5.8.5
    // CTRL9 Commands in Detail). On doing this the interrupt
    // pins will return to their normal function. After zeroing the
    // WoM Threshold the host processor may proceed to
    // reconfigure the QMI8658C as normal, as in the case
    // following a reset event.
    ConfigureCtrl7(module, &(Ctrl7Values) { });

    Ctrl9CalibrationData cal = 
    {
        .Registers = { (Ctrl9CalibrationRegisterData) { .Register = QMI8658Register_Cal1_L, .Value = 0 } },
        .RegisterCount = 1
    };

    RunCtrl9WriteCommand(module, QMI8658_Ctrl9_Cmd_WoM_Setting, &cal);
    
    // Reset accelerometer to previous configuration and reenable sensors.
    ConfigureAccelerometer(module, &module->AccelConfig);
    ConfigureCtrl7(
        module, 
        &(Ctrl7Values)
        { 
            .AttitudeEngineEnable = module->AttitudeEngineEnabled,
            .AccelerometerEnable = module->AccelConfig.Enabled,
            .GyroscopeEnable = module->GyroConfig.Enabled 
        });
}

static void DisposeResetAcknowledged(InterruptCallback* callback)
{
    RemoveInterruptCallback(callback, &callback->Module->Interrupt1Callbacks);
}

static void OnResetAcknowledged(InterruptCallback* callback)
{
    *((bool*)callback->Payload) = false;
    DisposeResetAcknowledged(callback);
}

static void Reset(Qmi8658* module)
{
    volatile bool waiting = true;
    AddInterruptCallback(
        CreateInterruptCallback(module, OnResetAcknowledged, (bool*)&waiting, DisposeAeDataEvent),
        &module->Interrupt1Callbacks);

    WriteSingle(module, QMI8658_Reset, 0xFF);

    while (waiting)
    {
        PollInt1(module);
    }

    sleep_ms(SYSTEM_TURN_ON_TIME_MILLIS);
}

static void PowerDown(Qmi8658* module)
{
    // CTRL1 sensorDisable = 1
    // CTRL7 aEN = 0, gEN = 0, mEN = 0, sEN=0. 

    // Disable sensors.
    ConfigureCtrl1(module, &(Ctrl1Values){ .DisableSensors = true });

    // Enter standby mode for all sensors.
    ConfigureCtrl7(module, &(Ctrl7Values){ });
    sleep_ms(SYSTEM_TURN_ON_TIME_MILLIS);
}

static inline bool InitI2c(i2c_inst_t* i2cInstance, Qmi8658* module)
{
    // I2C Config
    i2c_init(i2cInstance, 400 * 1000);
    gpio_set_function(DEV_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(DEV_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(DEV_SDA_PIN);
    gpio_pull_up(DEV_SCL_PIN);

    uint8_t chipId;
    uint8_t registerAddress = QMI8658Register_WhoAmI;
    i2c_write_blocking(module->I2cInstance, module->ModuleSlaveAddress, &registerAddress, 1, true);
    if (i2c_read_blocking(module->I2cInstance, module->ModuleSlaveAddress, &chipId, 1, false) == PICO_ERROR_GENERIC
        || chipId != 0x05) // The ID is a hardcoded identifier on the chip, always 0x05.
    {
        printf("Could not identify chip.\n");
        return false;
    }

    DumpConfig(module);

    Reset(module);
    ReadBytes(module, QMI8658Register_Revision, &module->ChipRevisionId, 1);

    ConfigureCtrl1(
        module,
        &(Ctrl1Values)
        {
            .SerialAutoincrement = true,
            .ReadBigEndian = true
        });

    return true;
}

void Qmi8658_Init(i2c_inst_t* i2cInstance, DeferredTaskScheduler* scheduler, Qmi8658* out)
{
    Qmi8658 module = 
    {
        .Reset = Reset,
        .ConfigureSensors = ConfigureSensors,
        .ConfigureAttitudeEngine = ConfigureAttitudeEngine,
        .EnableWakeOnMotion = EnableWakeOnMotion,
        .DisableWakeOnMotion = DisableWakeOnMotion,
        .PowerDown = PowerDown,
        .Scheduler = scheduler,
        .WakeOnMotionEnabled = false,
        .I2cInstance = i2cInstance,
        .ModuleSlaveAddress = QMI8658_SLAVE_ADDR_H
    };

    memcpy(out, &module, sizeof(Qmi8658));

    // AddInterruptCallback(
    //     CreateInterruptCallback(out, ReadInterruptStatus1, NULL),
    //     &int1Callback);

    // Enable INT1 interrupt.
    gpio_init(IMU_INT1_GPIO_PIN);
    gpio_set_dir(IMU_INT1_GPIO_PIN, GPIO_IN);
    gpio_pull_down(IMU_INT1_GPIO_PIN);
    gpio_set_irq_enabled(IMU_INT1_GPIO_PIN, GPIO_IRQ_EDGE_RISE, true);
    // gpio_set_irq_enabled(IMU_INT1_GPIO_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    // Enable INT2 interrupt.
    gpio_init(IMU_INT2_GPIO_PIN);
    gpio_set_dir(IMU_INT2_GPIO_PIN, GPIO_IN);
    gpio_pull_down(IMU_INT2_GPIO_PIN);
    gpio_set_irq_enabled(IMU_INT2_GPIO_PIN, GPIO_IRQ_EDGE_RISE, true);

    gpio_set_irq_callback(OnGpioInterruptReceived);
    irq_set_enabled(IO_IRQ_BANK0, true);

    InitI2c(i2cInstance, out);

    // DO NOT USE THIS METHOD! It only when for a single handler 
    // Per Pico hardware API docs:
    // "This method is commonly used to perform a one time setup, and following that 
    // any additional IRQs/events are enabled via gpio_set_irq_enabled. 
    // All GPIOs/events added in this way on the same core share the same callback; 
    // for multiple independent handlers for different GPIOs you should use gpio_add_raw_irq_handler and related functions."
    // gpio_set_irq_enabled_with_callback(
    //     IMU_INT1_GPIO_PIN, 
    //     GPIO_IRQ_EDGE_RISE, 
    //     true, 
    //     OnGpioInterruptReceived);
}