
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

void DEV_I2C_Write_Byte(uint8_t addr, uint8_t reg, uint8_t Value)
{
    uint8_t data[2] = {reg, Value};
    i2c_write_blocking(I2C_PORT, addr, data, 2, false);
}

void DEV_I2C_Read_nByte(uint8_t addr, uint8_t reg, uint8_t *pData, uint32_t Len)
{
    i2c_write_blocking(I2C_PORT, addr, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, addr, pData, Len, false);
}

static void ReadBytes(Qmi8658* module, uint8_t registerAddress, uint8_t *buffer, size_t registerReadCount)
{
    // First tell the module which register to read.
    i2c_write_blocking(module->I2cInstance, module->ModuleSlaveAddress, &registerAddress, 1, true);

    // Then read the contents, starting at the first register until `registerReadCount`.
    // Since we enabled autoincrement, it will automatically move to each register.
    i2c_read_blocking(module->I2cInstance, module->ModuleSlaveAddress, buffer, registerReadCount, false);
}

uint16_t DEC_ADC_Read(void)
{
    return adc_read();
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

// static void ReadCtrl6(Qmi8658* module, Ctrl6Values* out)
// {
// }

static void WriteCtrl6(Qmi8658* module, Ctrl6Values* out)
{
}

// static void ReadCtrl7(Qmi8658* module, Ctrl7Values* out)
// {
// }

static void WriteCtrl7(Qmi8658* module, Ctrl7Values* out)
{
}

struct InterruptCallback;
typedef void (*InterruptCallbackHandler)(struct InterruptCallback* callback, uint8_t status);

typedef struct InterruptCallback
{
    InterruptCallbackHandler OnInterruptReceived;
    void* Payload;
    struct InterruptCallback* Next;
}
InterruptCallback;

static InterruptCallback* CreateInterruptCallback(InterruptCallbackHandler onInterruptReceived, void* payload)
{
    InterruptCallback callback = 
    {
        .OnInterruptReceived = onInterruptReceived,
        .Payload = payload
    }; 

    InterruptCallback* out = (InterruptCallback*)malloc(sizeof(InterruptCallback));
    memcpy(out, &callback, sizeof(InterruptCallback));
    return out;
}

InterruptCallback* interrupt1Callbacks;

static void AddInterrupt1Callback(InterruptCallback* callback)
{
    InterruptCallback* next = interrupt1Callbacks;
    if (!next)
    {
        interrupt1Callbacks = callback;
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

static void RemoveInterruptCallback(InterruptCallback* callback)
{
    if (interrupt1Callbacks == callback)
    {
        interrupt1Callbacks = callback->Next; 
    }
    else
    {
        InterruptCallback* next = interrupt1Callbacks;
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

static void OnInterrupt1(uint gpio, uint32_t eventMask)
{
    if (gpio != IMU_INT1_GPIO_PIN)
    {
        // Not sure where this came from.
        return;
    }

    gpio_acknowledge_irq(gpio, eventMask);

    // Acknowledge interrupt and reset the status by reading.
    uint8_t status;
    QMI8658_read_reg(QMI8658Register_Status1, &status, 1);

    InterruptCallback* next = interrupt1Callbacks;
    while (next)
    {
        InterruptCallback* nextNext = next->Next;
        (*next->OnInterruptReceived)(next, status);
        next = nextNext;
    }
}

static void OnCtrl9CommandExecuted(InterruptCallback* callback, uint8_t status)
{
    // if (!(status & QMI8658_STATUS1_CTRL9_CMD_DONE))
    // {
    //     return;
    // }

    *((bool*)callback->Payload) = false;
    RemoveInterruptCallback(callback);
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
    
    RemoveInterruptCallback(callback);
    free(payload);
}

unsigned char QMI8658_write_reg(unsigned char reg, unsigned char value)
{
    unsigned char ret = 0;
    unsigned int retry = 0;

    while ((!ret) && (retry++ < 5))
    {
        DEV_I2C_Write_Byte(QMI8658_slave_addr, reg, value);
    }
    return ret;
}

unsigned char QMI8658_write_regs(unsigned char reg, unsigned char *value, unsigned char len)
{
    int i, ret;

    for (i = 0; i < len; i++)
    {
        ret = QMI8658_write_reg(reg + i, value[i]);
    }

    return ret;
}

unsigned char QMI8658_read_reg(unsigned char reg, unsigned char *buf, unsigned short len)
{
    unsigned char ret = 0;
    unsigned int retry = 0;
    DEV_I2C_Read_nByte(QMI8658_slave_addr, reg, buf, len);

    return ret;
}

void QMI8658_config_acc(enum QMI8658_AccRange range, enum QMI8658_AccOdr odr, enum QMI8658_LpfConfig lpfEnable, enum QMI8658_StConfig stEnable)
{
    unsigned char ctl_dada;

    switch (range)
    {
    case QMI8658AccRange_2g:
        acc_lsb_div = (1 << 14);
        break;
    case QMI8658AccRange_4g:
        acc_lsb_div = (1 << 13);
        break;
    case QMI8658AccRange_8g:
        acc_lsb_div = (1 << 12);
        break;
    case QMI8658AccRange_16g:
        acc_lsb_div = (1 << 11);
        break;
    default:
        range = QMI8658AccRange_8g;
        acc_lsb_div = (1 << 12);
    }
    if (stEnable == QMI8658St_Enable)
        ctl_dada = (unsigned char)range | (unsigned char)odr | 0x80;
    else
        ctl_dada = (unsigned char)range | (unsigned char)odr;

    QMI8658_write_reg(QMI8658Register_Ctrl2, ctl_dada);
    // set LPF & HPF
    QMI8658_read_reg(QMI8658Register_Ctrl5, &ctl_dada, 1);

    ctl_dada &= 0xf0;
    if (lpfEnable == QMI8658Lpf_Enable)
    {
        ctl_dada |= A_LSP_MODE_3;
        ctl_dada |= 0x01;
    }
    else
    {
        ctl_dada &= ~0x01;
    }
    ctl_dada = 0x00;
    QMI8658_write_reg(QMI8658Register_Ctrl5, ctl_dada);
    // set LPF & HPF
}

void QMI8658_config_gyro(enum QMI8658_GyrRange range, enum QMI8658_GyrOdr odr, enum QMI8658_LpfConfig lpfEnable, enum QMI8658_StConfig stEnable)
{
    // Set the CTRL3 register to configure dynamic range and ODR
    unsigned char ctl_dada;

    // Store the scale factor for use when processing raw data
    switch (range)
    {
    case QMI8658GyrRange_32dps:
        gyro_lsb_div = 1024;
        break;
    case QMI8658GyrRange_64dps:
        gyro_lsb_div = 512;
        break;
    case QMI8658GyrRange_128dps:
        gyro_lsb_div = 256;
        break;
    case QMI8658GyrRange_256dps:
        gyro_lsb_div = 128;
        break;
    case QMI8658GyrRange_512dps:
        gyro_lsb_div = 64;
        break;
    case QMI8658GyrRange_1024dps:
        gyro_lsb_div = 32;
        break;
    case QMI8658GyrRange_2048dps:
        gyro_lsb_div = 16;
        break;
    // case QMI8658GyrRange_4096dps:
    //     gyro_lsb_div = 8;
    //     break;
    default:
        range = QMI8658GyrRange_512dps;
        gyro_lsb_div = 64;
        break;
    }

    if (stEnable == QMI8658St_Enable)
        ctl_dada = (unsigned char)range | (unsigned char)odr | 0x80;
    else
        ctl_dada = (unsigned char)range | (unsigned char)odr;
    QMI8658_write_reg(QMI8658Register_Ctrl3, ctl_dada);

    // Conversion from degrees/s to rad/s if necessary
    // set LPF & HPF
    QMI8658_read_reg(QMI8658Register_Ctrl5, &ctl_dada, 1);
    ctl_dada &= 0x0f;
    if (lpfEnable == QMI8658Lpf_Enable)
    {
        ctl_dada |= G_LSP_MODE_3;
        ctl_dada |= 0x10;
    }
    else
    {
        ctl_dada &= ~0x10;
    }
    ctl_dada = 0x00;
    QMI8658_write_reg(QMI8658Register_Ctrl5, ctl_dada);
    // set LPF & HPF
}

void QMI8658_config_mag(enum QMI8658_MagDev device, enum QMI8658_MagOdr odr)
{
    QMI8658_write_reg(QMI8658Register_Ctrl4, device | odr);
}

void QMI8658_config_ae(enum QMI8658_AeOdr odr)
{
    // QMI8658_config_acc(QMI8658AccRange_8g, AccOdr_1000Hz, Lpf_Enable, St_Enable);
    // QMI8658_config_gyro(QMI8658GyrRange_2048dps, GyrOdr_1000Hz, Lpf_Enable, St_Enable);
    QMI8658_config_acc(QMI8658_config.accRange, QMI8658_config.accOdr, QMI8658Lpf_Enable, QMI8658St_Disable);
    QMI8658_config_gyro(QMI8658_config.gyrRange, QMI8658_config.gyrOdr, QMI8658Lpf_Enable, QMI8658St_Disable);
    QMI8658_config_mag(QMI8658_config.magDev, QMI8658_config.magOdr);
    QMI8658_write_reg(QMI8658Register_Ctrl6, odr);
}

unsigned char QMI8658_readStatus0(void)
{
    unsigned char status[2];

    QMI8658_read_reg(QMI8658Register_Status0, status, sizeof(status));
    // printf("status[0x%x	0x%x]\n",status[0],status[1]);

    return status[0];
}
/*!
 * \brief Blocking read of data status register 1 (::QMI8658Register_Status1).
 * \returns Status byte \see STATUS1 for flag definitions.
 */
unsigned char QMI8658_readStatus1(void)
{
    unsigned char status;

    QMI8658_read_reg(QMI8658Register_Status1, &status, sizeof(status));

    return status;
}

float QMI8658_readTemp(void)
{
    unsigned char buf[2];
    short temp = 0;
    float temp_f = 0;

    QMI8658_read_reg(QMI8658Register_Tempearture_L, buf, 2);
    temp = ((short)buf[1] << 8) | buf[0];
    temp_f = (float)temp / 256.0f;

    return temp_f;
}

unsigned int QMI8658_read_timestamp()
{
    unsigned char buf[3];
    QMI8658_read_reg(QMI8658Register_Timestamp_L, buf, 3); // 0x18	24
    unsigned int timestamp = (unsigned int)(((unsigned int)buf[2] << 16) | ((unsigned int)buf[1] << 8) | buf[0]);
    if (timestamp > imu_timestamp)
    {
        imu_timestamp = timestamp;
    }
    else
    {
        imu_timestamp = (timestamp + 0x1000000 - imu_timestamp);
    }

    return imu_timestamp;
}

static inline float ReadAccAxis(MotionDevice* device, short lo, short hi)
{
    return (float)(((short)((unsigned short)(hi << 8) | lo)) * 1000.0f) / device->LsbDivider;
}

static inline float ReadGyroAxis(MotionDevice* device, short lo, short hi)
{
    return (float)(((short)((unsigned short)(hi << 8) | lo)) * 1.0f) / device->LsbDivider;
}

// void QMI8658_read_acc_xyz(QMI8658_MotionCoordinates* acc)
// {
//     unsigned char registersBuffer[6];
//     QMI8658_read_reg(QMI8658Register_Ax_L, registersBuffer, 6); // 0x19, 25
//     acc->X = ReadAccAxis(registersBuffer[0], registersBuffer[1]);
//     acc->Y = ReadAccAxis(registersBuffer[2], registersBuffer[3]);
//     acc->Z = ReadAccAxis(registersBuffer[4], registersBuffer[5]);
// }

static void ReadAccelerometer(MotionDevice* device, QMI8658_MotionCoordinates* acc)
{
    unsigned char registersBuffer[6];
    ReadBytes(device->Module, QMI8658Register_Ax_L, registersBuffer, 6);
    acc->X = ReadAccAxis(device, registersBuffer[0], registersBuffer[1]);
    acc->Y = ReadAccAxis(device, registersBuffer[2], registersBuffer[3]);
    acc->Z = ReadAccAxis(device, registersBuffer[4], registersBuffer[5]);
}

// void QMI8658_read_gyro_xyz(QMI8658_MotionCoordinates* gyro)
// {
//     unsigned char registersBuffer[6];
//     QMI8658_read_reg(QMI8658Register_Gx_L, registersBuffer, 6); // 0x1f, 31
//     gyro->X = ReadGyroAxis(registersBuffer[0], registersBuffer[1]);
//     gyro->Y = ReadGyroAxis(registersBuffer[2], registersBuffer[3]);
//     gyro->Z = ReadGyroAxis(registersBuffer[4], registersBuffer[5]);
// }

static void ReadGyro(MotionDevice* device, QMI8658_MotionCoordinates* gyro)
{
    uint8_t registersBuffer[6];
    ReadBytes(device->Module, QMI8658Register_Gx_L, registersBuffer, 6); // 0x1f, 31
    gyro->X = ReadGyroAxis(device, registersBuffer[0], registersBuffer[1]);
    gyro->Y = ReadGyroAxis(device, registersBuffer[2], registersBuffer[3]);
    gyro->Z = ReadGyroAxis(device, registersBuffer[4], registersBuffer[5]);
}

// void QMI8658_read_xyz(QMI8658_MotionCoordinates* acc, QMI8658_MotionCoordinates* gyro)
// {
//     // ACC: Registers 53-58 (0x35 – 0x3A)
//     // GYR: Registers 59-64 (0x3B – 0x40)
//     // Start at register 53 and read 12 to get both in one shot.
//     unsigned char buf_reg[12];
//     QMI8658_read_reg(QMI8658Register_Ax_L, buf_reg, 12);
//     acc->X = ReadAccAxis(buf_reg[0], buf_reg[1]);
//     acc->Y = ReadAccAxis(buf_reg[2], buf_reg[3]);
//     acc->Z = ReadAccAxis(buf_reg[4], buf_reg[5]);
//     gyro->X = ReadGyroAxis(buf_reg[6], buf_reg[7]);
//     gyro->Y = ReadGyroAxis(buf_reg[8], buf_reg[9]);
//     gyro->Z = ReadGyroAxis(buf_reg[10], buf_reg[11]);
// }

static void ReadCombined(Qmi8658* module, QMI8658_MotionCoordinates* acc, QMI8658_MotionCoordinates* gyro)
{
    // ACC: Registers 53-58 (0x35 – 0x3A)
    // GYR: Registers 59-64 (0x3B – 0x40)
    // Start at register 53 and read 12 to get both in one shot.
    unsigned char registersBuffer[12];
    ReadBytes(module, QMI8658Register_Ax_L, registersBuffer, 12);
    acc->X = ReadAccAxis(&module->Accelerometer, registersBuffer[0], registersBuffer[1]);
    acc->Y = ReadAccAxis(&module->Accelerometer, registersBuffer[2], registersBuffer[3]);
    acc->Z = ReadAccAxis(&module->Accelerometer, registersBuffer[4], registersBuffer[5]);
    gyro->X = ReadGyroAxis(&module->Gyroscope, registersBuffer[6], registersBuffer[7]);
    gyro->Y = ReadGyroAxis(&module->Gyroscope, registersBuffer[8], registersBuffer[9]);
    gyro->Z = ReadGyroAxis(&module->Gyroscope, registersBuffer[10], registersBuffer[11]);
}

void QMI8658_read_xyz_raw(short raw_acc_xyz[3], short raw_gyro_xyz[3])
{
    unsigned char buf_reg[12];

    QMI8658_read_reg(QMI8658Register_Ax_L, buf_reg, 12); // 0x19, 25

    raw_acc_xyz[0] = (short)((unsigned short)(buf_reg[1] << 8) | (buf_reg[0]));
    raw_acc_xyz[1] = (short)((unsigned short)(buf_reg[3] << 8) | (buf_reg[2]));
    raw_acc_xyz[2] = (short)((unsigned short)(buf_reg[5] << 8) | (buf_reg[4]));

    raw_gyro_xyz[0] = (short)((unsigned short)(buf_reg[7] << 8) | (buf_reg[6]));
    raw_gyro_xyz[1] = (short)((unsigned short)(buf_reg[9] << 8) | (buf_reg[8]));
    raw_gyro_xyz[2] = (short)((unsigned short)(buf_reg[11] << 8) | (buf_reg[10]));
}

void QMI8658_read_ae(float quat[4], float velocity[3])
{
    unsigned char buf_reg[14];
    short raw_q_xyz[4];
    short raw_v_xyz[3];

    QMI8658_read_reg(QMI8658Register_Q1_L, buf_reg, 14);
    raw_q_xyz[0] = (short)((unsigned short)(buf_reg[1] << 8) | (buf_reg[0]));
    raw_q_xyz[1] = (short)((unsigned short)(buf_reg[3] << 8) | (buf_reg[2]));
    raw_q_xyz[2] = (short)((unsigned short)(buf_reg[5] << 8) | (buf_reg[4]));
    raw_q_xyz[3] = (short)((unsigned short)(buf_reg[7] << 8) | (buf_reg[6]));

    raw_v_xyz[1] = (short)((unsigned short)(buf_reg[9] << 8) | (buf_reg[8]));
    raw_v_xyz[2] = (short)((unsigned short)(buf_reg[11] << 8) | (buf_reg[10]));
    raw_v_xyz[2] = (short)((unsigned short)(buf_reg[13] << 8) | (buf_reg[12]));

    quat[0] = (float)(raw_q_xyz[0] * 1.0f) / ae_q_lsb_div;
    quat[1] = (float)(raw_q_xyz[1] * 1.0f) / ae_q_lsb_div;
    quat[2] = (float)(raw_q_xyz[2] * 1.0f) / ae_q_lsb_div;
    quat[3] = (float)(raw_q_xyz[3] * 1.0f) / ae_q_lsb_div;

    velocity[0] = (float)(raw_v_xyz[0] * 1.0f) / ae_v_lsb_div;
    velocity[1] = (float)(raw_v_xyz[1] * 1.0f) / ae_v_lsb_div;
    velocity[2] = (float)(raw_v_xyz[2] * 1.0f) / ae_v_lsb_div;
}

void QMI8658_read_mag(float mag[3])
{
    unsigned char buf_reg[6];
    short mag_xyz[3];

    QMI8658_read_reg(QMI8658Register_Mx_L, buf_reg, 6); // 0x1f, 31
    mag_xyz[0] = (short)((unsigned short)(buf_reg[1] << 8) | (buf_reg[0]));
    mag_xyz[1] = (short)((unsigned short)(buf_reg[3] << 8) | (buf_reg[2]));
    mag_xyz[2] = (short)((unsigned short)(buf_reg[5] << 8) | (buf_reg[4]));
    mag[0]=(float)mag_xyz[0];
    mag[1]=(float)mag_xyz[1];
    mag[2]=(float)mag_xyz[2];
}

static void RunCtrl9WriteCommand(enum QMI8658_Ctrl9Command command, CalibrationData* registers)
{
    for (unsigned int i = 0; i < registers->RegisterCount; i++)
    {
        CalibrationRegisterData* data = &registers->Registers[i];
        QMI8658_write_reg(data->Register, data->Value);
    }

    bool waiting = true;
    AddInterrupt1Callback(CreateInterruptCallback(OnCtrl9CommandExecuted, &waiting));

    QMI8658_write_reg(QMI8658Register_Ctrl9, command);

    while (waiting)
    {
        tight_loop_contents(); // NO OP
    }

    uint8_t status;
    QMI8658_read_reg(QMI8658Register_Status1, &status, 1);
}

void QMI8658_enableWakeOnMotion(Qmi8658* module, void (*onWake)(void* payload), void* callbackPayload)
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
    QMI8658_enableSensors(QMI8658_CTRL7_DISABLE_ALL);

    // 2. Set accelerometer sample rate and scale.
    QMI8658_config_acc(QMI8658AccRange_2g, QMI8658AccOdr_LowPower_21Hz, QMI8658Lpf_Disable, QMI8658St_Disable);

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
                .Value = (uint8_t)QMI8658WomThreshold_low
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

    RunCtrl9WriteCommand(QMI8658_Ctrl9_Cmd_WoM_Setting, &cal);
    
    QMI8658_enableSensors(QMI8658_CTRL7_ACC_ENABLE);
    sleep_ms(100);

    WomEventPayload* payload = (WomEventPayload*)malloc(sizeof(WomEventPayload));
    payload->Module = module;
    payload->Callback = onWake;
    payload->CallbackPayload = callbackPayload;
    AddInterrupt1Callback(CreateInterruptCallback(OnWomEvent, payload));
}

void QMI8658_disableWakeOnMotion(Qmi8658* module)
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
    QMI8658_enableSensors(QMI8658_CTRL7_DISABLE_ALL);

    CalibrationData cal = 
    {
        .Registers = { (CalibrationRegisterData) { .Register = QMI8658Register_Cal1_L, .Value = 0 } },
        .RegisterCount = 1
    };

    RunCtrl9WriteCommand(QMI8658_Ctrl9_Cmd_WoM_Setting, &cal);
    
    // Reenable sensors.
    QMI8658_setUpSensors();
}

void QMI8658_setUpSensors()
{
    QMI8658_write_reg(QMI8658Register_Ctrl1, CTRL1_SPI_AI_MASK | CTRL1_SPI_BE_MASK);
    QMI8658_config.inputSelection = QMI8658_CONFIG_GYR_ENABLE;
    // QMI8658_config.inputSelection = QMI8658_CONFIG_ACCGYR_ENABLE;
    QMI8658_config.accRange = QMI8658AccRange_8g;
    QMI8658_config.accOdr = QMI8658AccOdr_1000Hz;
    QMI8658_config.gyrRange = QMI8658GyrRange_512dps; 
    QMI8658_config.gyrOdr = QMI8658GyrOdr_1000Hz;
    // QMI8658_config.magOdr = QMI8658MagOdr_125Hz;
    // QMI8658_config.magDev = MagDev_AKM09918;
    // QMI8658_config.aeOdr = QMI8658AeOdr_128Hz;

    QMI8658_Config_apply(&QMI8658_config);
}

static inline void ConfigureAccelerometer(Qmi8658* module, Qmi8658AccelerometerConfig* config)
{
    switch (config->Range)
    {
        case QMI8658AccRange_2g:
            module->Accelerometer.LsbDivider = (1 << 14);
            break;
        case QMI8658AccRange_4g:
            module->Accelerometer.LsbDivider = (1 << 13);
            break;
        case QMI8658AccRange_8g:
            module->Accelerometer.LsbDivider = (1 << 12);
            break;
        case QMI8658AccRange_16g:
            module->Accelerometer.LsbDivider = (1 << 11);
            break;
        default:
            config->Range = QMI8658AccRange_8g;
            module->Accelerometer.LsbDivider = (1 << 12);
    }

    ConfigureCtrl2(
        module, 
        &(Ctrl2Values)
        {
            .SelfTest = config->SelfTestEnabled,
            .FullScale = config->Range,
            .OutputDataRate = config->Odr
        });

    // #define ACC_CONFIG_SELF_TEST_ENABLE 0b10000000

    // WriteSingle(
    //     module,
    //     QMI8658Register_Ctrl2, 
    //     config->SelfTestEnabled
    //         ? (uint8_t)config->Range | (uint8_t)config->Odr | ACC_CONFIG_SELF_TEST_ENABLE
    //         : (uint8_t)config->Range | (uint8_t)config->Odr);

    // uint8_t ctrl5Config = 0;
    // ReadBytes(module, QMI8658Register_Ctrl5, &ctrl5Config, 1);

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
}

static inline void ConfigureGyroscope(Qmi8658* module, Qmi8658GyroscopeConfig* config)
{
    // Store the scale factor for use when processing raw data
    switch (config->Range)
    {
        case QMI8658GyrRange_16dps:
            module->Gyroscope.LsbDivider = 2048;
            break;
        case QMI8658GyrRange_32dps:
            module->Gyroscope.LsbDivider = 1024;
            break;
        case QMI8658GyrRange_64dps:
            module->Gyroscope.LsbDivider = 512;
            break;
        case QMI8658GyrRange_128dps:
            module->Gyroscope.LsbDivider = 256;
            break;
        case QMI8658GyrRange_256dps:
            module->Gyroscope.LsbDivider = 128;
            break;
        case QMI8658GyrRange_512dps:
            module->Gyroscope.LsbDivider = 64;
            break;
        case QMI8658GyrRange_1024dps:
            module->Gyroscope.LsbDivider = 32;
            break;
        case QMI8658GyrRange_2048dps:
            module->Gyroscope.LsbDivider = 16;
            break;
        // case QMI8658GyrRange_4096dps:
        //     module->Gyroscope.LsbDivider = 8;
        //     break;
        default:
            config->Range = QMI8658GyrRange_512dps;
            module->Gyroscope.LsbDivider = 64;
            break;
    }

    ConfigureCtrl3(
        module,
        &(Ctrl3Values)
        {
            .SelfTest = config->SelfTestEnabled,
            .FullScale = config->Range,
            .OutputDataRate = config->Odr
        });

    // #define GYRO_CONFIG_SELF_TEST_ENABLE 0b10000000

    // WriteSingle(
    //     module, 
    //     QMI8658Register_Ctrl3, 
    //     config->SelfTestEnabled
    //         ? (uint8_t)config->Range | (uint8_t)config->Odr | GYRO_CONFIG_SELF_TEST_ENABLE
    //         : (uint8_t)config->Range | (uint8_t)config->Odr);

    uint8_t ctrl5Config = 0;
    ReadBytes(module, QMI8658Register_Ctrl5, &ctrl5Config, 1);

    #define CTRL5_GYRO_LPF_ENABLE 0b00010000
    if (config->LowPassFilterEnabled)
    {
        ctrl5Config |= G_LSP_MODE_3;
        ctrl5Config |= CTRL5_GYRO_LPF_ENABLE_MASK;
    }
    else
    {
        ctrl5Config &= ~0x10;
    }

    WriteSingle(module, QMI8658Register_Ctrl5, ctrl5Config);
}

static void ConfigureSensors(
    Qmi8658* module,
    bool enableAttitudeEngine, 
    Qmi8658AccelerometerConfig* accelConfig, 
    Qmi8658GyroscopeConfig* gyroConfig)
{
    ConfigureCtrl1(
        module,
        &(Ctrl1Values)
        {
            .SerialAutoincrement = true,
            .ReadBigEndian = true
        });
    // WriteSingle(module, QMI8658Register_Ctrl1, CTRL1_SPI_AI_MASK | CTRL1_SPI_BE_MASK); 

    // QMI8658_config_acc(QMI8658_config.accRange, QMI8658_config.accOdr, QMI8658Lpf_Enable, QMI8658St_Disable);
    // QMI8658_config_gyro(QMI8658_config.gyrRange, QMI8658_config.gyrOdr, QMI8658Lpf_Enable, QMI8658St_Disable);
    // QMI8658_config_mag(QMI8658_config.magDev, QMI8658_config.magOdr);

    // QMI8658_config.aeOdr = QMI8658AeOdr_128Hz;
    // QMI8658_write_reg(QMI8658Register_Ctrl6, odr);

    uint8_t ctrl7Enables = enableAttitudeEngine ? QMI8658_CTRL7_AE_ENABLE : 0;
    if (accelConfig && accelConfig->Enabled)
    {
        ctrl7Enables |= QMI8658_CONFIG_ACC_ENABLE;
        ConfigureAccelerometer(module, accelConfig);
        memcpy(&module->AccelConfig, accelConfig, sizeof(Qmi8658AccelerometerConfig));
    }

    if (gyroConfig && gyroConfig->Enabled)
    {
        ctrl7Enables |= QMI8658_CONFIG_GYR_ENABLE;
        ConfigureGyroscope(module, gyroConfig);
        memcpy(&module->GyroConfig, gyroConfig, sizeof(Qmi8658GyroscopeConfig));
    }

    // Enable the selected sensors.
    WriteSingle(module, QMI8658Register_Ctrl7, ctrl7Enables);
}

void QMI8658_enableSensors(unsigned char enableFlags)
{
    if (enableFlags & QMI8658_CONFIG_AE_ENABLE)
    {
        enableFlags |= QMI8658_CTRL7_ACC_ENABLE | QMI8658_CTRL7_GYR_ENABLE;
    }

    QMI8658_write_reg(QMI8658Register_Ctrl7, enableFlags & QMI8658_CTRL7_ENABLE_MASK);
}

void QMI8658_Config_apply(struct QMI8658Config const *config)
{
    unsigned char fisSensors = config->inputSelection;

    if (fisSensors & QMI8658_CONFIG_AE_ENABLE)
    {
        QMI8658_config_ae(config->aeOdr);
    }
    else
    {
        if (config->inputSelection & QMI8658_CONFIG_ACC_ENABLE)
        {
            QMI8658_config_acc(config->accRange, config->accOdr, QMI8658Lpf_Enable, QMI8658St_Disable);
        }
        if (config->inputSelection & QMI8658_CONFIG_GYR_ENABLE)
        {
            QMI8658_config_gyro(config->gyrRange, config->gyrOdr, QMI8658Lpf_Enable, QMI8658St_Disable);
        }
    }

    if (config->inputSelection & QMI8658_CONFIG_MAG_ENABLE)
    {
        QMI8658_config_mag(config->magDev, config->magOdr);
    }
    QMI8658_enableSensors(fisSensors);
}

unsigned char QMI8658_reset(void)
{
    QMI8658_write_reg(QMI8658_Reset, 0x01);
    sleep_ms(SYSTEM_TURN_ON_TIME_MILLIS);
}

static void Reset(Qmi8658* module)
{
    WriteSingle(module, QMI8658_Reset, 0x01);
    sleep_ms(SYSTEM_TURN_ON_TIME_MILLIS);
}

void QMI8658_powerDown(void)
{
    // CTRL1 sensorDisable = 1
    // CTRL7 aEN = 0, gEN = 0, mEN = 0, sEN=0. 

    // Disable sensors.
    QMI8658_write_reg(QMI8658Register_Ctrl1, CTRL1_SENSOR_DISABLE_MASK);

    // Enter standby mode for all sensors.
    QMI8658_write_reg(QMI8658Register_Ctrl1, 0);
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
        OnInterrupt1);

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
    return QMI8658_chip_id == 0x05;
}

void Qmi8658_Init(i2c_inst_t* i2cInstance, Qmi8658* out)
{
    Qmi8658 module = 
    {
        .Reset = Reset,
        .ConfigureSensors = ConfigureSensors,
        .Accelerometer = 
        {
            .Read = ReadAccelerometer,
            .Module = out
        },
        .Gyroscope = 
        {
            .Read = ReadGyro,
            .Module = out
        },
        .Sleeping = false,
        .WakeOnMotionEnabled = false,
        .I2cInstance = i2cInstance
    };

    InitI2c(i2cInstance, &module);
    memcpy(out, &module, sizeof(Qmi8658));
}