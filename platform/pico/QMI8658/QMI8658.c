
//#include "stdafx.h"
#include "QMI8658.h"
#include <string.h>

#define QMI8658_SLAVE_ADDR_L 0x6a
#define QMI8658_SLAVE_ADDR_H 0x6b
#define QMI8658_printf printf

#define QMI8658_UINT_MG_DPS

#define IMU_INT1_GPIO_PIN 23
#define IMU_INT2_GPIO_PIN 24

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

void DEV_I2C_Write_Register(uint8_t addr, uint8_t reg, uint16_t value)
{

    uint8_t tmpi[3];
    tmpi[0] = reg;
    tmpi[1] = (value >> 8) & 0xFF;
    tmpi[2] = value & 0xFF;
    DEV_I2C_Write_nByte(addr, tmpi, 3);
}

void DEV_I2C_Write_nByte(uint8_t addr, uint8_t *pData, uint32_t Len)
{
    i2c_write_blocking(I2C_PORT, addr, pData, Len, false);
}

uint8_t DEV_I2C_Read_Byte(uint8_t addr, uint8_t reg)
{
    uint8_t buf;
    i2c_write_blocking(I2C_PORT, addr, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, addr, &buf, 1, false);
    return buf;
}
void DEV_I2C_Read_Register(uint8_t addr, uint8_t reg, uint16_t *value)
{
    uint8_t tmpi[2];
    i2c_write_blocking(I2C_PORT, addr, &reg, 1, true); // true to keep master control of bus
    i2c_read_blocking(I2C_PORT, addr, tmpi, 2, false);
    *value = (((uint16_t)tmpi[0] << 8) | (uint16_t)tmpi[1]);
}

void DEV_I2C_Read_nByte(uint8_t addr, uint8_t reg, uint8_t *pData, uint32_t Len)
{
    i2c_write_blocking(I2C_PORT, addr, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, addr, pData, Len, false);
}

uint16_t DEC_ADC_Read(void)
{
    return adc_read();
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

    // free(callback);
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

    Qmi8658* module = callback->Payload;
    module->Sleeping = false;
    RemoveInterruptCallback(callback);
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
    case QMI8658GyrRange_4096dps:
        gyro_lsb_div = 8;
        break;
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

void QMI8658_read_acc_xyz(float acc_xyz[3])
{
    unsigned char buf_reg[6];
    short raw_acc_xyz[3];

    QMI8658_read_reg(QMI8658Register_Ax_L, buf_reg, 6); // 0x19, 25
    raw_acc_xyz[0] = (short)((unsigned short)(buf_reg[1] << 8) | (buf_reg[0]));
    raw_acc_xyz[1] = (short)((unsigned short)(buf_reg[3] << 8) | (buf_reg[2]));
    raw_acc_xyz[2] = (short)((unsigned short)(buf_reg[5] << 8) | (buf_reg[4]));

    acc_xyz[0] = (raw_acc_xyz[0] * ONE_G) / acc_lsb_div;
    acc_xyz[1] = (raw_acc_xyz[1] * ONE_G) / acc_lsb_div;
    acc_xyz[2] = (raw_acc_xyz[2] * ONE_G) / acc_lsb_div;

    // QMI8658_printf("fis210x acc:	%f	%f	%f\n", acc_xyz[0], acc_xyz[1], acc_xyz[2]);
}

void QMI8658_read_gyro_xyz(float gyro_xyz[3])
{
    unsigned char buf_reg[6];
    short raw_gyro_xyz[3];

    QMI8658_read_reg(QMI8658Register_Gx_L, buf_reg, 6); // 0x1f, 31
    raw_gyro_xyz[0] = (short)((unsigned short)(buf_reg[1] << 8) | (buf_reg[0]));
    raw_gyro_xyz[1] = (short)((unsigned short)(buf_reg[3] << 8) | (buf_reg[2]));
    raw_gyro_xyz[2] = (short)((unsigned short)(buf_reg[5] << 8) | (buf_reg[4]));

    gyro_xyz[0] = (raw_gyro_xyz[0] * 1.0f) / gyro_lsb_div;
    gyro_xyz[1] = (raw_gyro_xyz[1] * 1.0f) / gyro_lsb_div;
    gyro_xyz[2] = (raw_gyro_xyz[2] * 1.0f) / gyro_lsb_div;

    // QMI8658_printf("fis210x gyro:	%f	%f	%f\n", gyro_xyz[0], gyro_xyz[1], gyro_xyz[2]);
}

void QMI8658_read_xyz(QMI8658_MotionCoordinates* acc, QMI8658_MotionCoordinates* gyro, unsigned int *tim_count)
{
    unsigned char buf_reg[12];
    short raw_acc_xyz[3];
    short raw_gyro_xyz[3];
    //	float acc_t[3];
    //	float gyro_t[3];

    if (tim_count)
    {
        unsigned char buf[3];
        unsigned int timestamp;
        QMI8658_read_reg(QMI8658Register_Timestamp_L, buf, 3); // 0x18	24
        timestamp = (unsigned int)(((unsigned int)buf[2] << 16) | ((unsigned int)buf[1] << 8) | buf[0]);
        if (timestamp > imu_timestamp)
            imu_timestamp = timestamp;
        else
            imu_timestamp = (timestamp + 0x1000000 - imu_timestamp);

        *tim_count = imu_timestamp;
    }

    QMI8658_read_reg(QMI8658Register_Ax_L, buf_reg, 12); // 0x19, 25
    raw_acc_xyz[0] = (short)((unsigned short)(buf_reg[1] << 8) | (buf_reg[0]));
    raw_acc_xyz[1] = (short)((unsigned short)(buf_reg[3] << 8) | (buf_reg[2]));
    raw_acc_xyz[2] = (short)((unsigned short)(buf_reg[5] << 8) | (buf_reg[4]));

    raw_gyro_xyz[0] = (short)((unsigned short)(buf_reg[7] << 8) | (buf_reg[6]));
    raw_gyro_xyz[1] = (short)((unsigned short)(buf_reg[9] << 8) | (buf_reg[8]));
    raw_gyro_xyz[2] = (short)((unsigned short)(buf_reg[11] << 8) | (buf_reg[10]));

#if defined(QMI8658_UINT_MG_DPS)
    // mg
    acc->X = (float)(raw_acc_xyz[AXIS_X] * 1000.0f) / acc_lsb_div;
    acc->Y = (float)(raw_acc_xyz[AXIS_Y] * 1000.0f) / acc_lsb_div;
    acc->Z = (float)(raw_acc_xyz[AXIS_Z] * 1000.0f) / acc_lsb_div;
#else
    // m/s2
    acc[AXIS_X] = (float)(raw_acc_xyz[AXIS_X] * ONE_G) / acc_lsb_div;
    acc[AXIS_Y] = (float)(raw_acc_xyz[AXIS_Y] * ONE_G) / acc_lsb_div;
    acc[AXIS_Z] = (float)(raw_acc_xyz[AXIS_Z] * ONE_G) / acc_lsb_div;
#endif
    //	acc[AXIS_X] = imu_map.sign[AXIS_X]*acc_t[imu_map.map[AXIS_X]];
    //	acc[AXIS_Y] = imu_map.sign[AXIS_Y]*acc_t[imu_map.map[AXIS_Y]];
    //	acc[AXIS_Z] = imu_map.sign[AXIS_Z]*acc_t[imu_map.map[AXIS_Z]];

#if defined(QMI8658_UINT_MG_DPS)
    // dps
    gyro->X = (float)(raw_gyro_xyz[0] * 1.0f) / gyro_lsb_div;
    gyro->Y = (float)(raw_gyro_xyz[1] * 1.0f) / gyro_lsb_div;
    gyro->Z = (float)(raw_gyro_xyz[2] * 1.0f) / gyro_lsb_div;
#else
    // rad/s
    gyro[AXIS_X] = (float)(raw_gyro_xyz[AXIS_X] * 0.01745f) / gyro_lsb_div; // *pi/180
    gyro[AXIS_Y] = (float)(raw_gyro_xyz[AXIS_Y] * 0.01745f) / gyro_lsb_div;
    gyro[AXIS_Z] = (float)(raw_gyro_xyz[AXIS_Z] * 0.01745f) / gyro_lsb_div;
#endif
    //	gyro[AXIS_X] = imu_map.sign[AXIS_X]*gyro_t[imu_map.map[AXIS_X]];
    //	gyro[AXIS_Y] = imu_map.sign[AXIS_Y]*gyro_t[imu_map.map[AXIS_Y]];
    //	gyro[AXIS_Z] = imu_map.sign[AXIS_Z]*gyro_t[imu_map.map[AXIS_Z]];
}

void QMI8658_read_xyz_raw(short raw_acc_xyz[3], short raw_gyro_xyz[3], unsigned int *tim_count)
{
    unsigned char buf_reg[12];

    if (tim_count)
    {
        unsigned char buf[3];
        unsigned int timestamp;
        QMI8658_read_reg(QMI8658Register_Timestamp_L, buf, 3); // 0x18	24
        timestamp = (unsigned int)(((unsigned int)buf[2] << 16) | ((unsigned int)buf[1] << 8) | buf[0]);
        if (timestamp > imu_timestamp)
            imu_timestamp = timestamp;
        else
            imu_timestamp = (timestamp + 0x1000000 - imu_timestamp);

        *tim_count = imu_timestamp;
    }
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

void QMI8658_read_mag(float mag[3]){
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
        sleep_ms(1);
        // tight_loop_contents(); // NO OP
    }

    uint8_t status;
    QMI8658_read_reg(QMI8658Register_Status1, &status, 1);
}

void QMI8658_enableWakeOnMotion(Qmi8658* module)
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

    // 3. (A) Set wake on motion threshold.
    // WoM Threshold: absolute value in mg (with 1mg/LSB resolution).
    // QMI8658_write_reg(QMI8658Register_Cal1_L, (unsigned char)QMI8658WomThreshold_low); 

    // 3. (B) Select interrupt, polarity, and blanking time.
    const unsigned char blankingTimeMask = 0b00111111;

    // CAL1_H register:
    // 7:6 Interrupt select
    // 0:5 Interrupt blanking time (in number of accelerometer samples)
    // ------
    // Setting interrupt select to 00 - INT1 with initial value 0.
    // Setting blanking time to 4 samples.
    // QMI8658_write_reg(
    //     QMI8658Register_Cal1_H, 
    //     (unsigned char)QMI8658_Int1 | (unsigned char)QMI8658State_low | (0x04 & blankingTimeMask));

    // 4. Enable WOM through the matching CTRL9 command.
    // QMI8658_write_reg(QMI8658Register_Ctrl9, QMI8658_Ctrl9_Cmd_WoM_Setting);

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
                .Value = (uint8_t)QMI8658_Int1 | (uint8_t)QMI8658State_low | (0x04 & blankingTimeMask)
            },
        },
        .RegisterCount = 2
    };

    RunCtrl9WriteCommand(QMI8658_Ctrl9_Cmd_WoM_Setting, &cal);
    // sleep_ms(5);
    QMI8658_enableSensors(QMI8658_CTRL7_ACC_ENABLE);
    sleep_ms(100);

    AddInterrupt1Callback(CreateInterruptCallback(OnWomEvent, module));

    while (module->Sleeping)
    {
        sleep_ms(250);
    }
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
    QMI8658_write_reg(QMI8658Register_Cal1_L, 0);
    QMI8658_write_reg(QMI8658Register_Ctrl9, QMI8658_Ctrl9_Cmd_WoM_Setting);
}

void QMI8658_reenable()
{
    QMI8658_write_reg(QMI8658Register_Ctrl1, 0x60);
    QMI8658_config.inputSelection = QMI8658_CONFIG_ACCGYR_ENABLE; // QMI8658_CONFIG_ACCGYR_ENABLE;
    QMI8658_config.accRange = QMI8658AccRange_8g;
    QMI8658_config.accOdr = QMI8658AccOdr_1000Hz;
    QMI8658_config.gyrRange = QMI8658GyrRange_512dps; // QMI8658GyrRange_2048dps   QMI8658GyrRange_1024dps
    QMI8658_config.gyrOdr = QMI8658GyrOdr_1000Hz;
    QMI8658_config.magOdr = QMI8658MagOdr_125Hz;
    QMI8658_config.magDev = MagDev_AKM09918;
    QMI8658_config.aeOdr = QMI8658AeOdr_128Hz;
    QMI8658_Config_apply(&QMI8658_config);
    printf("QMI8658 reenabled\n");
}

void QMI8658_enableSensors(unsigned char enableFlags)
{
    if (enableFlags & QMI8658_CONFIG_AE_ENABLE){
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
}

unsigned char QMI8658_init(i2c_inst_t* i2cInstance, Qmi8658* out)
{
    Qmi8658 module = 
    {
        .Sleeping = false,
        .WakeOnMotionEnabled = false
    };

    memcpy(out, &module, sizeof(Qmi8658));

    // I2C Config
    i2c_init(i2cInstance, 400 * 1000);
    gpio_set_function(DEV_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(DEV_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(DEV_SDA_PIN);
    gpio_pull_up(DEV_SCL_PIN);

    gpio_init(IMU_INT1_GPIO_PIN);
    gpio_set_dir(IMU_INT1_GPIO_PIN, GPIO_IN);

    // TODO: Do we want a pull down?
    gpio_pull_down(IMU_INT1_GPIO_PIN);

    gpio_set_irq_enabled_with_callback(
        IMU_INT1_GPIO_PIN, 
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, 
        true, 
        OnInterrupt1);

    QMI8658_reset();
    sleep_ms(100);
    unsigned char QMI8658_chip_id = 0x00;
    unsigned char QMI8658_revision_id = 0x00;
    unsigned char QMI8658_slave[2] = {QMI8658_SLAVE_ADDR_L, QMI8658_SLAVE_ADDR_H};
    unsigned char iCount = 0;
    int retry = 0;

    while (iCount < 2)
    {
        QMI8658_slave_addr = QMI8658_slave[iCount];
        retry = 0;

        while ((QMI8658_chip_id != 0x05) && (retry++ < 5))
        {

            QMI8658_read_reg(QMI8658Register_WhoAmI, &QMI8658_chip_id, 1);
            // QMI8658_printf("QMI8658Register_WhoAmI = 0x%x\n", QMI8658_chip_id);
        }

        if (QMI8658_chip_id == 0x05)
        {
            break;
        }

        iCount++;
    }

    QMI8658_read_reg(QMI8658Register_Revision, &QMI8658_revision_id, 1);
    if (QMI8658_chip_id == 0x05)
    {
        // QMI8658_printf("QMI8658_init slave=0x%x  \r\nQMI8658Register_WhoAmI=0x%x 0x%x\n", QMI8658_slave_addr, QMI8658_chip_id, QMI8658_revision_id);
        QMI8658_write_reg(QMI8658Register_Ctrl1, 0x60);
        QMI8658_config.inputSelection = QMI8658_CONFIG_ACCGYR_ENABLE; // QMI8658_CONFIG_ACCGYR_ENABLE;
        QMI8658_config.accRange = QMI8658AccRange_8g;
        QMI8658_config.accOdr = QMI8658AccOdr_1000Hz;
        QMI8658_config.gyrRange = QMI8658GyrRange_512dps; // QMI8658GyrRange_2048dps   QMI8658GyrRange_1024dps
        QMI8658_config.gyrOdr = QMI8658GyrOdr_1000Hz;
        QMI8658_config.magOdr = QMI8658MagOdr_125Hz;
        QMI8658_config.magDev = MagDev_AKM09918;
        QMI8658_config.aeOdr = QMI8658AeOdr_128Hz;

        QMI8658_Config_apply(&QMI8658_config);
        // if (1)
        // {
        //     unsigned char read_data = 0x00;
        //     QMI8658_read_reg(QMI8658Register_Ctrl1, &read_data, 1);
        //     QMI8658_printf("QMI8658Register_Ctrl1=0x%x \n", read_data);
        //     QMI8658_read_reg(QMI8658Register_Ctrl2, &read_data, 1);
        //     QMI8658_printf("QMI8658Register_Ctrl2=0x%x \n", read_data);
        //     QMI8658_read_reg(QMI8658Register_Ctrl3, &read_data, 1);
        //     QMI8658_printf("QMI8658Register_Ctrl3=0x%x \n", read_data);
        //     QMI8658_read_reg(QMI8658Register_Ctrl4, &read_data, 1);
        //     QMI8658_printf("QMI8658Register_Ctrl4=0x%x \n", read_data);
        //     QMI8658_read_reg(QMI8658Register_Ctrl5, &read_data, 1);
        //     QMI8658_printf("QMI8658Register_Ctrl5=0x%x \n", read_data);
        //     QMI8658_read_reg(QMI8658Register_Ctrl6, &read_data, 1);
        //     QMI8658_printf("QMI8658Register_Ctrl6=0x%x \n", read_data);
        //     QMI8658_read_reg(QMI8658Register_Ctrl7, &read_data, 1);
        //     QMI8658_printf("QMI8658Register_Ctrl7=0x%x \n", read_data);
        // }
        //		QMI8658_set_layout(2);
        return 1;
    }
    else
    {
        // QMI8658_printf("QMI8658_init fail\n");
        QMI8658_chip_id = 0;
        return 0;
    }
    // return QMI8658_chip_id;
}
