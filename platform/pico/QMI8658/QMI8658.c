
//#include "stdafx.h"
#include "QMI8658.h"
#include <string.h>

#define QMI8658_Reset 0x60

#define QMI8658_SLAVE_ADDR_L 0x6a
#define QMI8658_SLAVE_ADDR_H 0x6b
#define QMI8658_printf printf

#define IMU_INT1_GPIO_PIN 23
#define IMU_INT2_GPIO_PIN 24

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

typedef struct 
{
    Qmi8658* Module;
    void (*Callback)(void* payload);
    void* CallbackPayload;
}
WomEventPayload;

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

enum
{
    AXIS_X = 0,
    AXIS_Y = 1,
    AXIS_Z = 2,

    AXIS_TOTAL
};

typedef struct
{
    short sign[AXIS_TOTAL];
    unsigned short map[AXIS_TOTAL];
} qst_imu_layout;

typedef struct 
{
    enum QMI8658Register Register;
    uint8_t Value;
}
CalibrationRegisterData;

typedef struct  
{
    CalibrationRegisterData Registers[8];
    unsigned int RegisterCount;
}
CalibrationData;

static unsigned short acc_lsb_div = 0;
static unsigned short gyro_lsb_div = 0;
static unsigned short ae_q_lsb_div = (1 << 14);
static unsigned short ae_v_lsb_div = (1 << 10);
static unsigned int imu_timestamp = 0;
static struct QMI8658Config QMI8658_config;
static unsigned char QMI8658_slave_addr = QMI8658_SLAVE_ADDR_L;

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
        return I2cReadResult_AddressWriteFailed;
    }

    // Then read the contents, starting at the first register until `registerReadCount`.
    // Since we enabled autoincrement, it will automatically move to each register.
    result = i2c_read_blocking(module->I2cInstance, module->ModuleSlaveAddress, buffer, registerReadCount, false);
    return result == registerReadCount
        ? I2cReadResult_Success
        : result > 0 && result != PICO_ERROR_GENERIC
            ? I2cReadResult_ReadPartial
            : I2cReadResult_ReadFailed;
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

    memcpy(out, &values, sizeof(Ctrl1Values));
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

// gST
#define CTRL3_GYRO_SELF_TEST_MASK 0b10000000

// gFS
#define CTRL3_GYRO_FULL_SCALE_MASK 0b01110000

// gODR
#define CTRL2_GYRO_ODR_MASK 0b00001111

static void ConfigureCtrl3(Qmi8658* module, Ctrl3Values* config)
{
    WriteSingle(
        module, 
        QMI8658Register_Ctrl3, 
        (config->SelfTest ? CTRL3_GYRO_SELF_TEST_MASK : 0)
            | config->FullScale
            | config->OutputDataRate);
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

    memcpy(out, &values, sizeof(Ctrl1Values));
}

// sMoD
#define CTRL6_MOD_ENABLE_MASK 0b10000000

// sODR
#define CTRL6_AE_ODR_ENABLE_MASK 0b00000111

static void ConfigureCtrl6(Qmi8658* module, Ctrl6Values* config)
{    
    WriteSingle(
        module, 
        QMI8658Register_Ctrl6, 
        (config->MotionOnDemand ? CTRL6_MOD_ENABLE_MASK : 0)
            | config->AttitudeEngineOutputDataRate);
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

typedef struct GpioInterruptCallback
{
    void (*OnInterruptReceived)(Qmi8658* module);
    Qmi8658* Module;
}
GpioInterruptCallback;

GpioInterruptCallback gpioInterruptCallback;

static InterruptCallback* CreateInterruptCallback(Qmi8658* module, InterruptCallbackHandler onInterruptReceived, void* payload)
{
    InterruptCallback callback = 
    {
        .OnInterruptReceived = onInterruptReceived,
        .Payload = payload,
        .Module = module
    }; 

    InterruptCallback* out = (InterruptCallback*)malloc(sizeof(InterruptCallback));
    memcpy(out, &callback, sizeof(InterruptCallback));
    return out;
}

static void AddInterrupt1Callback(Qmi8658* module, InterruptCallback* callback)
{
    InterruptCallback* next = module->Interrupt1Callbacks;
    if (!next)
    {
        module->Interrupt1Callbacks = callback;
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

static void RemoveInterruptCallback(Qmi8658* module, InterruptCallback* callback)
{
    if (module->Interrupt1Callbacks == callback)
    {
        module->Interrupt1Callbacks = callback->Next; 
    }
    else
    {
        InterruptCallback* next = module->Interrupt1Callbacks;
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

static void OnInterrupt1(Qmi8658* module)
{
    // Acknowledge interrupt and reset the status by reading.
    uint8_t status;
    ReadBytes(module, QMI8658Register_Status1, &status, 1);

    InterruptCallback* next = module->Interrupt1Callbacks;
    while (next)
    {
        InterruptCallback* nextNext = next->Next;
        (*next->OnInterruptReceived)(next, status);
        next = nextNext;
    }
}

static void OnGpioInterrupt1Received(uint gpio, uint32_t eventMask)
{
    if (gpio != IMU_INT1_GPIO_PIN)
    {
        // Not sure where this came from.
        return;
    }

    gpio_acknowledge_irq(gpio, eventMask);
    (*gpioInterruptCallback.OnInterruptReceived)(gpioInterruptCallback.Module);
}

static void OnCtrl9CommandExecuted(InterruptCallback* callback, uint8_t status)
{
    // if (!(status & QMI8658_STATUS1_CTRL9_CMD_DONE))
    // {
    //     return;
    // }

    *((bool*)callback->Payload) = false;
    RemoveInterruptCallback(callback->Module, callback);
}

static void OnWomEvent(InterruptCallback* callback, uint8_t status)
{
    if (!(status & QMI8658_STATUS1_WAKEUP_EVENT))
    {
        return;
    }

    WomEventPayload* payload = (WomEventPayload*)callback->Payload;
    payload->Module->Sleeping = false;
    if (payload->Callback)
    {
        (*payload->Callback)(payload->CallbackPayload);
    }
    
    RemoveInterruptCallback(callback->Module, callback);
    free(payload);
}

// void QMI8658_config_mag(enum QMI8658_MagDev device, enum QMI8658_MagOdr odr)
// {
//     QMI8658_write_reg(QMI8658Register_Ctrl4, device | odr);
// }

// void QMI8658_config_ae(enum QMI8658_AeOdr odr)
// {
//     // QMI8658_config_acc(QMI8658AccRange_8g, AccOdr_1000Hz, Lpf_Enable, St_Enable);
//     // QMI8658_config_gyro(QMI8658GyrRange_2048dps, GyrOdr_1000Hz, Lpf_Enable, St_Enable);
//     QMI8658_config_acc(QMI8658_config.accRange, QMI8658_config.accOdr, QMI8658Lpf_Enable, QMI8658St_Disable);
//     QMI8658_config_gyro(QMI8658_config.gyrRange, QMI8658_config.gyrOdr, QMI8658Lpf_Enable, QMI8658St_Disable);
//     QMI8658_config_mag(QMI8658_config.magDev, QMI8658_config.magOdr);
//     QMI8658_write_reg(QMI8658Register_Ctrl6, odr);
// }

// float QMI8658_readTemp(void)
// {
//     unsigned char buf[2];
//     short temp = 0;
//     float temp_f = 0;

//     QMI8658_read_reg(QMI8658Register_Tempearture_L, buf, 2);
//     temp = ((short)buf[1] << 8) | buf[0];
//     temp_f = (float)temp / 256.0f;

//     return temp_f;
// }

// unsigned int QMI8658_read_timestamp()
// {
//     unsigned char buf[3];
//     QMI8658_read_reg(QMI8658Register_Timestamp_L, buf, 3); // 0x18	24
//     unsigned int timestamp = (unsigned int)(((unsigned int)buf[2] << 16) | ((unsigned int)buf[1] << 8) | buf[0]);
//     if (timestamp > imu_timestamp)
//     {
//         imu_timestamp = timestamp;
//     }
//     else
//     {
//         imu_timestamp = (timestamp + 0x1000000 - imu_timestamp);
//     }

//     return imu_timestamp;
// }

static inline float ReadAccAxis(MotionDevice* device, short lo, short hi)
{
    return (float)(((short)((unsigned short)(hi << 8) | lo)) * 1000.0f) / device->LsbDivider;
}

static inline float ReadGyroAxis(MotionDevice* device, short lo, short hi)
{
    return (float)(((short)((unsigned short)(hi << 8) | lo)) * 1.0f) / device->LsbDivider;
}

static void ReadAccelerometer(MotionDevice* device, QMI8658_MotionCoordinates* acc)
{
    unsigned char registersBuffer[6] = { };
    ReadBytes(device->Module, QMI8658Register_Ax_L, registersBuffer, 6);
    acc->X = ReadAccAxis(device, registersBuffer[0], registersBuffer[1]);
    acc->Y = ReadAccAxis(device, registersBuffer[2], registersBuffer[3]);
    acc->Z = ReadAccAxis(device, registersBuffer[4], registersBuffer[5]);
}

static void ReadGyro(MotionDevice* device, QMI8658_MotionCoordinates* gyro)
{
    uint8_t registersBuffer[6] = { };
    ReadBytes(device->Module, QMI8658Register_Gx_L, registersBuffer, 6); // 0x1f, 31
    gyro->X = ReadGyroAxis(device, registersBuffer[0], registersBuffer[1]);
    gyro->Y = ReadGyroAxis(device, registersBuffer[2], registersBuffer[3]);
    gyro->Z = ReadGyroAxis(device, registersBuffer[4], registersBuffer[5]);
}

// static void ReadCombined(Qmi8658* module, QMI8658_MotionCoordinates* acc, QMI8658_MotionCoordinates* gyro)
// {
//     // ACC: Registers 53-58 (0x35 – 0x3A)
//     // GYR: Registers 59-64 (0x3B – 0x40)
//     // Start at register 53 and read 12 to get both in one shot.
//     unsigned char registersBuffer[12];
//     ReadBytes(module, QMI8658Register_Ax_L, registersBuffer, 12);
//     acc->X = ReadAccAxis(&module->Accelerometer, registersBuffer[0], registersBuffer[1]);
//     acc->Y = ReadAccAxis(&module->Accelerometer, registersBuffer[2], registersBuffer[3]);
//     acc->Z = ReadAccAxis(&module->Accelerometer, registersBuffer[4], registersBuffer[5]);
//     gyro->X = ReadGyroAxis(&module->Gyroscope, registersBuffer[6], registersBuffer[7]);
//     gyro->Y = ReadGyroAxis(&module->Gyroscope, registersBuffer[8], registersBuffer[9]);
//     gyro->Z = ReadGyroAxis(&module->Gyroscope, registersBuffer[10], registersBuffer[11]);
// }

// void QMI8658_read_ae(float quat[4], float velocity[3])
// {
//     unsigned char buf_reg[14];
//     short raw_q_xyz[4];
//     short raw_v_xyz[3];

//     QMI8658_read_reg(QMI8658Register_Q1_L, buf_reg, 14);
//     raw_q_xyz[0] = (short)((unsigned short)(buf_reg[1] << 8) | (buf_reg[0]));
//     raw_q_xyz[1] = (short)((unsigned short)(buf_reg[3] << 8) | (buf_reg[2]));
//     raw_q_xyz[2] = (short)((unsigned short)(buf_reg[5] << 8) | (buf_reg[4]));
//     raw_q_xyz[3] = (short)((unsigned short)(buf_reg[7] << 8) | (buf_reg[6]));

//     raw_v_xyz[1] = (short)((unsigned short)(buf_reg[9] << 8) | (buf_reg[8]));
//     raw_v_xyz[2] = (short)((unsigned short)(buf_reg[11] << 8) | (buf_reg[10]));
//     raw_v_xyz[2] = (short)((unsigned short)(buf_reg[13] << 8) | (buf_reg[12]));

//     quat[0] = (float)(raw_q_xyz[0] * 1.0f) / ae_q_lsb_div;
//     quat[1] = (float)(raw_q_xyz[1] * 1.0f) / ae_q_lsb_div;
//     quat[2] = (float)(raw_q_xyz[2] * 1.0f) / ae_q_lsb_div;
//     quat[3] = (float)(raw_q_xyz[3] * 1.0f) / ae_q_lsb_div;

//     velocity[0] = (float)(raw_v_xyz[0] * 1.0f) / ae_v_lsb_div;
//     velocity[1] = (float)(raw_v_xyz[1] * 1.0f) / ae_v_lsb_div;
//     velocity[2] = (float)(raw_v_xyz[2] * 1.0f) / ae_v_lsb_div;
// }

// void QMI8658_read_mag(float mag[3])
// {
//     unsigned char buf_reg[6];
//     short mag_xyz[3];

//     QMI8658_read_reg(QMI8658Register_Mx_L, buf_reg, 6); // 0x1f, 31
//     mag_xyz[0] = (short)((unsigned short)(buf_reg[1] << 8) | (buf_reg[0]));
//     mag_xyz[1] = (short)((unsigned short)(buf_reg[3] << 8) | (buf_reg[2]));
//     mag_xyz[2] = (short)((unsigned short)(buf_reg[5] << 8) | (buf_reg[4]));
//     mag[0]=(float)mag_xyz[0];
//     mag[1]=(float)mag_xyz[1];
//     mag[2]=(float)mag_xyz[2];
// }

static void RunCtrl9WriteCommand(Qmi8658* module, enum QMI8658_Ctrl9Command command, CalibrationData* registers)
{
    for (unsigned int i = 0; i < registers->RegisterCount; i++)
    {
        CalibrationRegisterData* data = &registers->Registers[i];
        WriteSingle(module, data->Register, data->Value);
    }

    bool waiting = true;
    AddInterrupt1Callback(module, CreateInterruptCallback(module, OnCtrl9CommandExecuted, &waiting));

    WriteSingle(module, QMI8658Register_Ctrl9, command);

    while (waiting)
    {
        tight_loop_contents(); // NO OP
    }

    // uint8_t status;
    // ReadBytes(module, QMI8658Register_Status1, &status, 1);
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
                .LsbDivider = ConfigureAccelerometer(module, accelConfig)
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
                .LsbDivider = ConfigureGyroscope(module, gyroConfig)
            }, 
            sizeof(MotionDevice));
    }

    // Enable the selected sensors.
    ConfigureCtrl7(
        module, 
        &(Ctrl7Values)
        { 
            .AttitudeEngineEnable = false,
            .AccelerometerEnable = accelConfig->Enabled,
            .GyroscopeEnable = gyroConfig->Enabled 
        });

    sleep_us(startupTimeUs);

    module->AttitudeEngineEnabled = false;
    memcpy(&module->AccelConfig, accelConfig, sizeof(Qmi8658AccelerometerConfig));
    memcpy(&module->GyroConfig, gyroConfig, sizeof(Qmi8658GyroscopeConfig));
}

static void EnableWakeOnMotion(Qmi8658* module, void (*onWake)(void* payload), void* callbackPayload)
{
    module->WakeOnMotionEnabled = true;
    module->Sleeping = true;

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
    CalibrationData cal = 
    {
        .Registers = 
        {
            // WoM Threshold: absolute value in mg (with 1mg/LSB resolution).
            (CalibrationRegisterData) 
            { 
                .Register = QMI8658Register_Cal1_L,
                .Value = (uint8_t)QMI8658WomThreshold_high
            },
            (CalibrationRegisterData) 
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
    // sleep_ms(100);

    WomEventPayload* payload = (WomEventPayload*)malloc(sizeof(WomEventPayload));
    payload->Module = module;
    payload->Callback = onWake;
    payload->CallbackPayload = callbackPayload;
    AddInterrupt1Callback(module, CreateInterruptCallback(module, OnWomEvent, payload));
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

    CalibrationData cal = 
    {
        .Registers = { (CalibrationRegisterData) { .Register = QMI8658Register_Cal1_L, .Value = 0 } },
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

static void Reset(Qmi8658* module)
{
    WriteSingle(module, QMI8658_Reset, 0x01);
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

    // Enable INT1 interrupt.
    gpio_init(IMU_INT1_GPIO_PIN);
    gpio_set_dir(IMU_INT1_GPIO_PIN, GPIO_IN);
    gpio_pull_down(IMU_INT1_GPIO_PIN);
    gpio_set_irq_enabled_with_callback(
        IMU_INT1_GPIO_PIN, 
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, 
        true, 
        OnGpioInterrupt1Received);

    unsigned char QMI8658_chip_id = 0x00;
    unsigned char QMI8658_slave[2] = { QMI8658_SLAVE_ADDR_L, QMI8658_SLAVE_ADDR_H };
    unsigned char iCount = 0;
    while (iCount < 2)
    {
        module->ModuleSlaveAddress = QMI8658_slave_addr = QMI8658_slave[iCount];

        unsigned int retry = 0;
        while ((QMI8658_chip_id != 0x05) && (retry++ < 5))
        {
            ReadBytes(module, QMI8658Register_WhoAmI, &QMI8658_chip_id, 1);
        }

        if (QMI8658_chip_id == 0x05)
        {
            break;
        }

        iCount++;
    }

    ReadBytes(module, QMI8658Register_Revision, &module->ChipRevisionId, 1);

    ConfigureCtrl1(
        module,
        &(Ctrl1Values)
        {
            .SerialAutoincrement = true,
            .ReadBigEndian = true
        });

    return QMI8658_chip_id == 0x05;
}

void Qmi8658_Init(i2c_inst_t* i2cInstance, Qmi8658* out)
{
    Qmi8658 module = 
    {
        .Reset = Reset,
        .ConfigureSensors = ConfigureSensors,
        .EnableWakeOnMotion = EnableWakeOnMotion,
        .DisableWakeOnMotion = DisableWakeOnMotion,
        // .ReadCombined = ReadCombined,
        .PowerDown = PowerDown,
        // .Accelerometer = 
        // {
        //     .Read = ReadAccelerometer,
        //     .Module = out
        // },
        // .Gyroscope = 
        // {
        //     .Read = ReadGyro,
        //     .Module = out
        // },
        .Sleeping = false,
        .WakeOnMotionEnabled = false,
        .I2cInstance = i2cInstance
    };

    InitI2c(i2cInstance, &module);

    gpioInterruptCallback.Module = out;
    gpioInterruptCallback.OnInterruptReceived = OnInterrupt1;

    memcpy(out, &module, sizeof(Qmi8658));
}