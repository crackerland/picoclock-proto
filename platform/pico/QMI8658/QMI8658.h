// QMI8658C
// Low Noise, Wide Bandwidth 6D Inertial Measurement
// Unit with Motion CoProcessor and Sensor Fusion

#ifndef QMI8658_H
#define QMI8658_H

//#include "DEV_Config.h"
#include <stdint.h>
#include <stdlib.h> //itoa()
#include <stdio.h>
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "TaskScheduler.h"

#define UBYTE uint8_t
#define UWORD uint16_t
#define UDOUBLE uint32_t

#define SPI_PORT spi1
#define I2C_PORT i2c1
/**
 * GPIOI config
 **/

#define LCD_DC_PIN 8
#define LCD_CS_PIN 9
#define LCD_CLK_PIN 10
#define LCD_MOSI_PIN 11
#define LCD_RST_PIN 12
#define LCD_BL_PIN 25

#define DEV_SDA_PIN     (6)
#define DEV_SCL_PIN     (7)

#define BAT_ADC_PIN     (29)
#define BAR_CHANNEL     (3)




#ifndef M_PI
#define M_PI (3.14159265358979323846f)
#endif
#ifndef ONE_G
#define ONE_G (9.807f)
#endif

#define QMI8658_CTRL7_DISABLE_ALL (0x0)
#define QMI8658_CTRL7_ACC_ENABLE (0x1)
#define QMI8658_CTRL7_GYR_ENABLE (0x2)
#define QMI8658_CTRL7_MAG_ENABLE (0x4)
#define QMI8658_CTRL7_AE_ENABLE (0x8)
#define QMI8658_CTRL7_GYR_SNOOZE_ENABLE (0x10)
#define QMI8658_CTRL7_ENABLE_MASK (0xF)

#define QMI8658_CONFIG_ACC_ENABLE QMI8658_CTRL7_ACC_ENABLE
#define QMI8658_CONFIG_GYR_ENABLE QMI8658_CTRL7_GYR_ENABLE
#define QMI8658_CONFIG_MAG_ENABLE QMI8658_CTRL7_MAG_ENABLE
#define QMI8658_CONFIG_AE_ENABLE QMI8658_CTRL7_AE_ENABLE
#define QMI8658_CONFIG_ACCGYR_ENABLE (QMI8658_CONFIG_ACC_ENABLE | QMI8658_CONFIG_GYR_ENABLE)
#define QMI8658_CONFIG_ACCMAG_ENABLE (QMI8658_CONFIG_ACC_ENABLE | QMI8658_CONFIG_MAG_ENABLE)
#define QMI8658_CONFIG_ACCGYRMAG_ENABLE (QMI8658_CONFIG_ACC_ENABLE | QMI8658_CONFIG_GYR_ENABLE | QMI8658_CONFIG_MAG_ENABLE)
#define QMI8658_CONFIG_AEMAG_ENABLE (QMI8658_CONFIG_AE_ENABLE | QMI8658_CONFIG_MAG_ENABLE)

enum QMI8658Register
{
    /*! \brief FIS device identifier register. */
    QMI8658Register_WhoAmI = 0, // 0
    /*! \brief FIS hardware revision register. */
    QMI8658Register_Revision, // 1
    /*! \brief General and power management modes. */
    QMI8658Register_Ctrl1, // 2
    /*! \brief Accelerometer control. */
    QMI8658Register_Ctrl2, // 3
    /*! \brief Gyroscope control. */
    QMI8658Register_Ctrl3, // 4
    /*! \brief Magnetometer control. */
    QMI8658Register_Ctrl4, // 5
    /*! \brief Data processing settings. */
    QMI8658Register_Ctrl5, // 6
    /*! \brief AttitudeEngine control. */
    QMI8658Register_Ctrl6, // 7
    /*! \brief Sensor enabled status. */
    QMI8658Register_Ctrl7, // 8
    /*! \brief Reserved - do not write. */
    QMI8658Register_Ctrl8, // 9
    /*! \brief Host command register. */
    QMI8658Register_Ctrl9, // 10
    /*! \brief Calibration register 1 most significant byte. */
    QMI8658Register_Cal1_L = 11,
    /*! \brief Calibration register 1 least significant byte. */
    QMI8658Register_Cal1_H,
    /*! \brief Calibration register 2 most significant byte. */
    QMI8658Register_Cal2_L,
    /*! \brief Calibration register 2 least significant byte. */
    QMI8658Register_Cal2_H,
    /*! \brief Calibration register 3 most significant byte. */
    QMI8658Register_Cal3_L,
    /*! \brief Calibration register 3 least significant byte. */
    QMI8658Register_Cal3_H,
    /*! \brief Calibration register 4 most significant byte. */
    QMI8658Register_Cal4_L,
    /*! \brief Calibration register 4 least significant byte. */
    QMI8658Register_Cal4_H,
    /*! \brief FIFO control register. */
    QMI8658Register_FifoCtrl = 19,
    /*! \brief FIFO data register. */
    QMI8658Register_FifoData, // 20
    /*! \brief FIFO status register. */
    QMI8658Register_FifoStatus, // 21
    /*! \brief I2C Master Status. */
    QMI8658Register_I2cmStatus = 44,
    /*! \brief Interrupt status. */
    QMI8658Register_StatusInt = 45,
    /*! \brief Output data overrun and availability. */
    QMI8658Register_Status0 = 46,
    /*! \brief Miscellaneous status register. */
    QMI8658Register_Status1 = 47,
    /*! \brief timestamp low. */
    QMI8658Register_Timestamp_L = 48,
    /*! \brief timestamp low. */
    QMI8658Register_Timestamp_M,
    /*! \brief timestamp low. */
    QMI8658Register_Timestamp_H,
    /*! \brief tempearture low. */
    QMI8658Register_Tempearture_L = 51,
    /*! \brief tempearture low. */
    QMI8658Register_Tempearture_H,
    /*! \brief Accelerometer X axis least significant byte. */
    QMI8658Register_Ax_L = 53,
    /*! \brief Accelerometer X axis most significant byte. */
    QMI8658Register_Ax_H,
    /*! \brief Accelerometer Y axis least significant byte. */
    QMI8658Register_Ay_L,
    /*! \brief Accelerometer Y axis most significant byte. */
    QMI8658Register_Ay_H,
    /*! \brief Accelerometer Z axis least significant byte. */
    QMI8658Register_Az_L,
    /*! \brief Accelerometer Z axis most significant byte. */
    QMI8658Register_Az_H,
    /*! \brief Gyroscope X axis least significant byte. */
    QMI8658Register_Gx_L = 59,
    /*! \brief Gyroscope X axis most significant byte. */
    QMI8658Register_Gx_H,
    /*! \brief Gyroscope Y axis least significant byte. */
    QMI8658Register_Gy_L,
    /*! \brief Gyroscope Y axis most significant byte. */
    QMI8658Register_Gy_H,
    /*! \brief Gyroscope Z axis least significant byte. */
    QMI8658Register_Gz_L,
    /*! \brief Gyroscope Z axis most significant byte. */
    QMI8658Register_Gz_H,
    /*! \brief Magnetometer X axis least significant byte. */
    QMI8658Register_Mx_L = 65,
    /*! \brief Magnetometer X axis most significant byte. */
    QMI8658Register_Mx_H,
    /*! \brief Magnetometer Y axis least significant byte. */
    QMI8658Register_My_L,
    /*! \brief Magnetometer Y axis most significant byte. */
    QMI8658Register_My_H,
    /*! \brief Magnetometer Z axis least significant byte. */
    QMI8658Register_Mz_L,
    /*! \brief Magnetometer Z axis most significant byte. */
    QMI8658Register_Mz_H,
    /*! \brief Quaternion increment W least significant byte. */
    QMI8658Register_Q1_L = 73,
    /*! \brief Quaternion increment W most significant byte. */
    QMI8658Register_Q1_H,
    /*! \brief Quaternion increment X least significant byte. */
    QMI8658Register_Q2_L,
    /*! \brief Quaternion increment X most significant byte. */
    QMI8658Register_Q2_H,
    /*! \brief Quaternion increment Y least significant byte. */
    QMI8658Register_Q3_L,
    /*! \brief Quaternion increment Y most significant byte. */
    QMI8658Register_Q3_H,
    /*! \brief Quaternion increment Z least significant byte. */
    QMI8658Register_Q4_L,
    /*! \brief Quaternion increment Z most significant byte. */
    QMI8658Register_Q4_H,
    /*! \brief Velocity increment X least significant byte. */
    QMI8658Register_Dvx_L = 81,
    /*! \brief Velocity increment X most significant byte. */
    QMI8658Register_Dvx_H,
    /*! \brief Velocity increment Y least significant byte. */
    QMI8658Register_Dvy_L,
    /*! \brief Velocity increment Y most significant byte. */
    QMI8658Register_Dvy_H,
    /*! \brief Velocity increment Z least significant byte. */
    QMI8658Register_Dvz_L,
    /*! \brief Velocity increment Z most significant byte. */
    QMI8658Register_Dvz_H,
    /*! \brief AttitudeEngine reg1. */
    QMI8658Register_AeReg1 = 87,
    /*! \brief AttitudeEngine overflow flags. */
    QMI8658Register_AeOverflow,
};

enum QMI8658_Ois_Register
{
    /*-----------------------------*/
    /* Setup and Control Registers */
    /*-----------------------------*/
    /*! \brief SPI Endian Selection, and SPI 3/4 Wire */
    QMI8658_OIS_Reg_Ctrl1 = 0x02, // 2  [0x02] -- Dflt: 0x20
    /*! \brief Accelerometer control: ODR, Full Scale, Self Test  */
    QMI8658_OIS_Reg_Ctrl2, //  3  [0x03]
    /*! \brief Gyroscope control: ODR, Full Scale, Self Test */
    QMI8658_OIS_Reg_Ctrl3, //  4  [0x04]
    /*! \brief Sensor Data Processing Settings */
    QMI8658_OIS_Reg_Ctrl5 = 0x06, //  6  [0x06]
    /*! \brief Sensor enabled status: Enable Sensors  */
    QMI8658_OIS_Reg_Ctrl7 = 0x08, //  8  [0x08]
    /*-------------------*/
    /* Status Registers  */
    /*-------------------*/
    /*! \brief Sensor Data Availability and Lock Register */
    QMI8658_OIS_Reg_StatusInt = 0x2D, // 45  [0x2D]
    /*! \brief Output data overrun and availability */
    QMI8658_OIS_Reg_Status0 = 0x2E, // 46  [0x2E]

    /*-----------------------------------------------------*/
    /* OIS Sensor Data Output Registers. 16-bit 2's complement */
    /*-----------------------------------------------------*/
    /*! \brief Accelerometer X axis least significant byte */
    QMI8658_OIS_Reg_Ax_L = 0x33, // 53  [0x35]
    /*! \brief Accelerometer X axis most significant byte */
    QMI8658_OIS_Reg_Ax_H, // 54  [0x36]
    /*! \brief Accelerometer Y axis least significant byte */
    QMI8658_OIS_Reg_Ay_L, // 55  [0x37]
    /*! \brief Accelerometer Y axis most significant byte */
    QMI8658_OIS_Reg_Ay_H, // 56  [0x38]
    /*! \brief Accelerometer Z axis least significant byte */
    QMI8658_OIS_Reg_Az_L, // 57  [0x39]
    /*! \brief Accelerometer Z axis most significant byte */
    QMI8658_OIS_Reg_Az_H, // 58  [0x3A]

    /*! \brief Gyroscope X axis least significant byte */
    QMI8658_OIS_Reg_Gx_L = 0x3B, // 59  [0x3B]
    /*! \brief Gyroscope X axis most significant byte */
    QMI8658_OIS_Reg_Gx_H, // 60  [0x3C]
    /*! \brief Gyroscope Y axis least significant byte */
    QMI8658_OIS_Reg_Gy_L, // 61  [0x3D]
    /*! \brief Gyroscope Y axis most significant byte */
    QMI8658_OIS_Reg_Gy_H, // 62  [0x3E]
    /*! \brief Gyroscope Z axis least significant byte */
    QMI8658_OIS_Reg_Gz_L, // 63  [0x3F]
    /*! \brief Gyroscope Z axis most significant byte */
    QMI8658_OIS_Reg_Gz_H, // 64  [0x40]
};

enum QMI8658_Ctrl9Command
{
    QMI8658_Ctrl9_Cmd_NOP = 0X00,
    QMI8658_Ctrl9_Cmd_GyroBias = 0X01,

    // CTRL_CMD_REQ_SDI | 0x03 | Ctrl9R | SDI MOD (Motion on Demand), request to read SDI data
    QMI8658_Ctrl9_Cmd_Rqst_Sdi_Mod = 0X03,
    QMI8658_Ctrl9_Cmd_WoM_Setting = 0x08,
    QMI8658_Ctrl9_Cmd_AccelHostDeltaOffset = 0x09,
    QMI8658_Ctrl9_Cmd_GyroHostDeltaOffset = 0x0A,
    QMI8658_Ctrl9_Cmd_Dbg_WoM_Data_Enable = 0xF8,
};

enum QMI8658_LpfConfig
{
    QMI8658Lpf_Disable, /*!< \brief Disable low pass filter. */
    QMI8658Lpf_Enable   /*!< \brief Enable low pass filter. */
};

enum QMI8658_HpfConfig
{
    QMI8658Hpf_Disable, /*!< \brief Disable high pass filter. */
    QMI8658Hpf_Enable   /*!< \brief Enable high pass filter. */
};

enum QMI8658_StConfig
{
    QMI8658St_Disable, /*!< \brief Disable high pass filter. */
    QMI8658St_Enable   /*!< \brief Enable high pass filter. */
};

enum QMI8658_LpfMode
{
    // Accelerometer low pass filter 2.62% of ODR.
    A_LSP_MODE_0 = 0x00 << 1,

    // Accelerometer low pass filter 3.59% of ODR.
    A_LSP_MODE_1 = 0x01 << 1,

    // Accelerometer low pass filter 5.32% of ODR.
    A_LSP_MODE_2 = 0x02 << 1,

    // Accelerometer low pass filter 14.0% of ODR.
    A_LSP_MODE_3 = 0x03 << 1,

    // Gyro low pass filter 2.62% of ODR.
    G_LSP_MODE_0 = 0x00 << 5,

    // Gyro low pass filter 3.59% of ODR.
    G_LSP_MODE_1 = 0x01 << 5,

    // Gyro low pass filter 5.32% of ODR.
    G_LSP_MODE_2 = 0x02 << 5,

    // Gyro low pass filter 14.0% of ODR.
    G_LSP_MODE_3 = 0x03 << 5
};

enum QMI8658_AccRange
{
    QMI8658AccRange_2g = 0x00 << 4, /*!< \brief +/- 2g range */
    QMI8658AccRange_4g = 0x01 << 4, /*!< \brief +/- 4g range */
    QMI8658AccRange_8g = 0x02 << 4, /*!< \brief +/- 8g range */
    QMI8658AccRange_16g = 0x03 << 4 /*!< \brief +/- 16g range */
};

enum QMI8658_AccOdr
{
    QMI8658AccOdr_8000Hz = 0x00,         /*!< \brief High resolution 8000Hz output rate. */
    QMI8658AccOdr_4000Hz = 0x01,         /*!< \brief High resolution 4000Hz output rate. */
    QMI8658AccOdr_2000Hz = 0x02,         /*!< \brief High resolution 2000Hz output rate. */
    QMI8658AccOdr_1000Hz = 0x03,         /*!< \brief High resolution 1000Hz output rate. */
    QMI8658AccOdr_500Hz = 0x04,          /*!< \brief High resolution 500Hz output rate. */
    QMI8658AccOdr_250Hz = 0x05,          /*!< \brief High resolution 250Hz output rate. */
    QMI8658AccOdr_125Hz = 0x06,          /*!< \brief High resolution 125Hz output rate. */
    QMI8658AccOdr_62_5Hz = 0x07,         /*!< \brief High resolution 62.5Hz output rate. */
    QMI8658AccOdr_31_25Hz = 0x08,        /*!< \brief High resolution 31.25Hz output rate. */
    QMI8658AccOdr_LowPower_128Hz = 0x0c, /*!< \brief Low power 128Hz output rate. */
    QMI8658AccOdr_LowPower_21Hz = 0x0d,  /*!< \brief Low power 21Hz output rate. */
    QMI8658AccOdr_LowPower_11Hz = 0x0e,  /*!< \brief Low power 11Hz output rate. */
    QMI8658AccOdr_LowPower_3Hz = 0x0f    /*!< \brief Low power 3Hz output rate. */
};

enum QMI8658_GyrRange
{
    QMI8658GyrRange_16dps = 0b000 << 4,   /*!< \brief +-32 degrees per second. */
    QMI8658GyrRange_32dps = 0b001 << 4,   /*!< \brief +-32 degrees per second. */
    QMI8658GyrRange_64dps = 0b010 << 4,   /*!< \brief +-64 degrees per second. */
    QMI8658GyrRange_128dps = 0b011 << 4,  /*!< \brief +-128 degrees per second. */
    QMI8658GyrRange_256dps = 0b100 << 4,  /*!< \brief +-256 degrees per second. */
    QMI8658GyrRange_512dps = 0b101 << 4,  /*!< \brief +-512 degrees per second. */
    QMI8658GyrRange_1024dps = 0b110 << 4, /*!< \brief +-1024 degrees per second. */
    QMI8658GyrRange_2048dps = 0b111 << 4, /*!< \brief +-2048 degrees per second. */

    // This is not shown in the datasheet. Why is it here?
    // QMI8658GyrRange_4096dps = 8 << 4  /*!< \brief +-2560 degrees per second. */
};

/*!
 * \brief Gyroscope output rate configuration.
 */
enum QMI8658_GyrOdr
{
    QMI8658GyrOdr_8000Hz = 0x00, /*\brief High resolution 8000Hz output rate. */
    QMI8658GyrOdr_4000Hz = 0x01, /*\brief High resolution 4000Hz output rate. */
    QMI8658GyrOdr_2000Hz = 0x02, /*\brief High resolution 2000Hz output rate. */
    QMI8658GyrOdr_1000Hz = 0x03, /*\brief High resolution 1000Hz output rate. */
    QMI8658GyrOdr_500Hz = 0x04,  /*\brief High resolution 500Hz output rate. */
    QMI8658GyrOdr_250Hz = 0x05,  /*\brief High resolution 250Hz output rate. */
    QMI8658GyrOdr_125Hz = 0x06,  /*\brief High resolution 125Hz output rate. */
    QMI8658GyrOdr_62_5Hz = 0x07, /*\brief High resolution 62.5Hz output rate. */
    QMI8658GyrOdr_31_25Hz = 0x08 /*\brief High resolution 31.25Hz output rate. */
};

enum QMI8658_AeOdr
{
    QMI8658AeOdr_1Hz = 0x00,   /*!< \brief 1Hz output rate. */
    QMI8658AeOdr_2Hz = 0x01,   /*!< \brief 2Hz output rate. */
    QMI8658AeOdr_4Hz = 0x02,   /*!< \brief 4Hz output rate. */
    QMI8658AeOdr_8Hz = 0x03,   /*!< \brief 8Hz output rate. */
    QMI8658AeOdr_16Hz = 0x04,  /*!< \brief 16Hz output rate. */
    QMI8658AeOdr_32Hz = 0x05,  /*!< \brief 32Hz output rate. */
    QMI8658AeOdr_64Hz = 0x06,  /*!< \brief 64Hz output rate. */
    // QMI8658AeOdr_128Hz = 0x07, /*!< \brief 128Hz output rate. */
    /*!
     * \brief Motion on demand mode.
     *
     * In motion on demand mode the application can trigger AttitudeEngine
     * output samples as necessary. This allows the AttitudeEngine to be
     * synchronized with external data sources.
     *
     * When in Motion on Demand mode the application should request new data
     * by calling the QMI8658_requestAttitudeEngineData() function. The
     * AttitudeEngine will respond with a data ready event (INT2) when the
     * data is available to be read.
     */
    // QMI8658AeOdr_motionOnDemand = 128
};

enum QMI8658_MagOdr
{
    QMI8658MagOdr_1000Hz = 0x00, /*!< \brief 1000Hz output rate. */
    QMI8658MagOdr_500Hz = 0x01,  /*!< \brief 500Hz output rate. */
    QMI8658MagOdr_250Hz = 0x02,  /*!< \brief 250Hz output rate. */
    QMI8658MagOdr_125Hz = 0x03,  /*!< \brief 125Hz output rate. */
    QMI8658MagOdr_62_5Hz = 0x04, /*!< \brief 62.5Hz output rate. */
    QMI8658MagOdr_31_25Hz = 0x05 /*!< \brief 31.25Hz output rate. */
};

enum QMI8658_MagDev
{
    MagDev_AKM09918 = (0 << 3), /*!< \brief AKM09918. */
};

enum QMI8658_AccUnit
{
    QMI8658AccUnit_g,  /*!< \brief Accelerometer output in terms of g (9.81m/s^2). */
    QMI8658AccUnit_ms2 /*!< \brief Accelerometer output in terms of m/s^2. */
};

enum QMI8658_GyrUnit
{
    QMI8658GyrUnit_dps, /*!< \brief Gyroscope output in degrees/s. */
    QMI8658GyrUnit_rads /*!< \brief Gyroscope output in rad/s. */
};

struct QMI8658Config
{
    /*! \brief Sensor fusion input selection. */
    unsigned char inputSelection;
    /*! \brief Accelerometer dynamic range configuration. */
    enum QMI8658_AccRange accRange;
    /*! \brief Accelerometer output rate. */
    enum QMI8658_AccOdr accOdr;
    /*! \brief Gyroscope dynamic range configuration. */
    enum QMI8658_GyrRange gyrRange;
    /*! \brief Gyroscope output rate. */
    enum QMI8658_GyrOdr gyrOdr;
    /*! \brief AttitudeEngine output rate. */
    enum QMI8658_AeOdr aeOdr;
    /*!
     * \brief Magnetometer output data rate.
     *
     * \remark This parameter is not used when using an external magnetometer.
     * In this case the external magnetometer is sampled at the FIS output
     * data rate, or at an integer divisor thereof such that the maximum
     * sample rate is not exceeded.
     */
    enum QMI8658_MagOdr magOdr;

    /*!
     * \brief Magnetometer device to use.
     *
     * \remark This parameter is not used when using an external magnetometer.
     */
    enum QMI8658_MagDev magDev;
};

#define QMI8658_SAMPLE_SIZE (3 * sizeof(short))
#define QMI8658_AE_SAMPLE_SIZE ((4 + 3 + 1) * sizeof(short) + sizeof(unsigned char))
struct FisImuRawSample
{
    /*! \brief The sample counter of the sample. */
    unsigned char timestamp[3];
    /*!
     * \brief Pointer to accelerometer data in the sample buffer.
     *
     * \c NULL if no accelerometer data is available in the buffer.
     */
    unsigned char const *accelerometerData;
    /*!
     * \brief Pointer to gyroscope data in the sample buffer.
     *
     * \c NULL if no gyroscope data is available in the buffer.
     */
    unsigned char const *gyroscopeData;
    /*!
     * \brief Pointer to magnetometer data in the sample buffer.
     *
     * \c NULL if no magnetometer data is available in the buffer.
     */
    unsigned char const *magnetometerData;
    /*!
     * \brief Pointer to AttitudeEngine data in the sample buffer.
     *
     * \c NULL if no AttitudeEngine data is available in the buffer.
     */
    unsigned char const *attitudeEngineData;
    /*! \brief Raw sample buffer. */
    unsigned char sampleBuffer[QMI8658_SAMPLE_SIZE + QMI8658_AE_SAMPLE_SIZE];
    /*! \brief Contents of the FIS status 1 register. */
    unsigned char status1;
    // unsigned char status0;
    // unsigned int durT;
};

struct QMI8658_offsetCalibration
{
    enum QMI8658_AccUnit accUnit;
    float accOffset[3];
    enum QMI8658_GyrUnit gyrUnit;
    float gyrOffset[3];
};

struct QMI8658_sensitivityCalibration
{
    float accSensitivity[3];
    float gyrSensitivity[3];
};

enum QMI8658_Interrupt
{
    /*! \brief FIS INT1 line. */
    QMI8658_Int1 = (0 << 6),
    /*! \brief FIS INT2 line. */
    QMI8658_Int2 = (1 << 6)
};

enum QMI8658_InterruptState
{
    QMI8658State_high = (1 << 7), /*!< Interrupt high. */
    QMI8658State_low = (0 << 7)   /*!< Interrupt low. */
};

enum QMI8658_WakeOnMotionThreshold
{
    QMI8658WomThreshold_high = 128, /*!< High threshold - large motion needed to wake. */
    QMI8658WomThreshold_low = 32    /*!< Low threshold - small motion needed to wake. */
};

// Describes a vector or right-handed cartesian coordinates, depending on how the 
// source device is configured.
typedef struct Qmi8658_MotionVector
{
    float X;
    float Y;
    float Z;
}
Qmi8658_MotionVector;

typedef struct Qmi8658_Quaternion
{
    float W;
    float X;
    float Y;
    float Z;
}
Qmi8658_Quaternion;

struct Qmi8658;

typedef struct MotionDevice
{
    void (*Read)(struct MotionDevice*, Qmi8658_MotionVector* vectorOut, Qmi8658_Quaternion* quaternionOut);
    unsigned short VectorLsbDivider;
    unsigned short QuaternionLsbDivider;
    struct Qmi8658* Module;
} 
MotionDevice;

typedef struct 
{
    bool Enabled;
    enum QMI8658_AccRange Range; 
    enum QMI8658_AccOdr Odr; 
    bool LowPassFilterEnabled; 
    bool SelfTestEnabled;
}
Qmi8658AccelerometerConfig;

typedef struct 
{
    bool Enabled;
    enum QMI8658_GyrRange Range; 
    enum QMI8658_GyrOdr Odr; 
    bool LowPassFilterEnabled; 
    bool SelfTestEnabled;
}
Qmi8658GyroscopeConfig;

typedef struct Status0Values
{
    // (sDA) Whether the Attitude Engine output registers have data avilable.
    bool AttitudeEngineDataAvailable;

    // (mDA) Whether the external magnetometer output registers have data avilable.
    bool MagnetometerDataAvailable;
    
    // (gDA) Whether the gyroscope output registers have data avilable.
    bool GyroscopeDataAvailable;

    // (aDA) Whether the accelerometer output registers have data avilable.
    bool AccelerometerDataAvailable;
}
Status0Values;

typedef struct Status1Values
{
    // (reserved - See 9.3) Whether a "Wake on Motion" event has been raised.
    bool WakeOnMotionEvent;

    // (CmdDone) Whether a CTRL9 command has completed.
    bool Ctrl9CommandDone;
}
Status1Values;

struct InterruptCallback;
struct Qmi8658;
typedef void (*InterruptCallbackHandler)(
    struct InterruptCallback* callback);
    // struct Status0Values* status0,
    // struct Status1Values* status1);

typedef struct InterruptCallback
{
    InterruptCallbackHandler OnInterruptReceived;
    bool Complete;
    void* Payload;
    struct InterruptCallback* Next;
    struct Qmi8658* Module;
    void (*Dispose)(struct InterruptCallback*);
}
InterruptCallback;

typedef struct Qmi8658
{
    // All QMI8658C functional blocks are switched off to
    // minimize power consumption. Digital interfaces remain on
    // allowing communication with the device. All configuration
    // register values are preserved, and output data register
    // values are maintained. The current in this mode is typically
    // 20 µA. The host must initiate this mode by setting
    // sensorDisable=1.
    void (*PowerDown)(struct Qmi8658*);
    void (*Reset)(struct Qmi8658*);
    void (*ConfigureSensors)(
        struct Qmi8658*,
        Qmi8658AccelerometerConfig* accelConfig, 
        Qmi8658GyroscopeConfig* gyroConfig,
        MotionDevice* accelerometerOut,
        MotionDevice* gyroscopeOut);

    void (*ConfigureAttitudeEngine)(
        struct Qmi8658*,
        bool enableMotionOnDemand,
        enum QMI8658_AeOdr dataRate,
        MotionDevice* attitudeEngineOut);

    InterruptCallback* (*EnableWakeOnMotion)(struct Qmi8658*, void (*onWake)(void* payload), void* callbackPayload);
    void (*DisableWakeOnMotion)(struct Qmi8658*);
    DeferredTaskScheduler* Scheduler;
    bool WakeOnMotionEnabled;
    i2c_inst_t* I2cInstance;
    uint8_t ModuleSlaveAddress;
    uint8_t ChipRevisionId;
    bool AttitudeEngineEnabled;
    Qmi8658AccelerometerConfig AccelConfig;
    Qmi8658GyroscopeConfig GyroConfig;
    InterruptCallback* Interrupt1Callbacks;
    InterruptCallback* Interrupt2Callbacks;
}
Qmi8658;

extern void Qmi8658_Init(i2c_inst_t* i2cInstance, DeferredTaskScheduler* scheduler, Qmi8658* out);

#endif
