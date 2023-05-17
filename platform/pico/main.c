#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "QMI8658.h"
#include "PicoClock.h"
#include "PicoTimer.h"
#include "PicoDateTimeProvider.h"
#include "PicoPowerManager.h"
#include "PicoBattery.h"
#include "DefaultDateTimeFormatter.h"
#include "pico/stdlib.h"
#include "GC9A01A.h"
#include "ByteHelper.h"
#include "Color.h"
#include "DateTimeFormatter.h"
#include "SystemFontTextView.h"
#include "ScreenTextureRenderer.h"
#include "BufferTextureRenderer.h"
#include "Texture16.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/flash.h"

extern _Colors Colors;

#define SAVE_STATE_CONTENT_SIZE (FLASH_PAGE_SIZE - 5)
typedef struct SaveState
{
    char Name[5];
    uint8_t Content[SAVE_STATE_CONTENT_SIZE];
}
SaveState;

static void SetUpGpioPin(uint16_t pin, bool direction)
{
    gpio_init(pin);
    gpio_set_dir(pin, direction);
}

// Allocate a single sector (which is the minimum) at the end of the flash memory
// for user data. Program data is at the beginning, so this will not conflict.
#define FLASH_USER_DATA_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
const uint8_t* flashContents = (const uint8_t*) (XIP_BASE + FLASH_USER_DATA_OFFSET);
static SaveState* LoadFlashState()
{
    return (SaveState*)flashContents;
}

static void SaveFlashState(uint8_t* data, size_t size)
{
    SaveState state = 
    {
        .Name = "SAVE"
    };

    memcpy(&state.Content, data, size);

    // A whole number of sectors must be erased at a time.
    flash_range_erase(FLASH_USER_DATA_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_USER_DATA_OFFSET, (uint8_t*)&state, FLASH_PAGE_SIZE);
}

static PicoDateTimeProvider dateTimeProvider;
static datetime_t pendingTime = { };
static bool updateTime = false;

// RX interrupt handler
void on_uart_rx() 
{
    uint8_t buffer[11] = { };
    uart_read_blocking(uart0, buffer, sizeof(uint8_t) * 10);

    buffer[10] = '\0';

    int now = (int)atoi(buffer);
    now -= 60 * 60 * 4; // Offset UTC - 4 hours to make EST.
    time_t nowBuffer = (time_t)now;
    struct tm* parsed = localtime(&nowBuffer);

    pendingTime.year = parsed->tm_year + 1900; // `tm_year` is the number of years since 1900.
    pendingTime.month = parsed->tm_mon;
    pendingTime.day = parsed->tm_mday;
    pendingTime.dotw = parsed->tm_wday;
    pendingTime.hour = parsed->tm_hour;
    pendingTime.min = parsed->tm_min;
    pendingTime.sec = parsed->tm_sec;

    updateTime = true;
}

// static void OnTimeout(Timer*, void*);
// static void ResetTimeout(AppResources* resources)
// {
//     (*resources->Timer->Post)(resources->Timer, 5000000, resources, OnTimeout);
// }

// static bool sleep = false;
// static void OnTimeout(Timer* timer, void* param)
// {
//     AppResources* app = (AppResources*)param;
//     sleep = true;
//     ResetTimeout(app);
// }

int main(void)
{
    stdio_init_all();

    uart_init(uart0, 115200);
    gpio_set_function(1, GPIO_FUNC_UART);

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
    irq_set_enabled(UART0_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(uart0, true, false);

    PicoDateTimeProvider_Init(&dateTimeProvider);

    PicoTimer timer;
    PicoTimer_Init(&timer);

    LcdScreen lcdScreen = { };
    LcdScreen_Init(&timer.Base, ScanDirection_Horizontal, &lcdScreen);

    PicoPowerManager powerManager;
    PicoPowerManager_Init(&lcdScreen, &powerManager);

    App app;
    App_Init(&lcdScreen.Base, &timer.Base, &dateTimeProvider.Base, &powerManager.Base, &app);

    QMI8658_init(i2c1);
    uint32_t lastMovementTime = time_us_32(); 
    bool lowPowerMode = false;
    float acc[3] = { };
    float gyro[3] = { };
    unsigned int tim_count = 0;
    while(1)
    {
        if (updateTime)
        {
            if (!rtc_set_datetime(&pendingTime))
            {
                printf("ERROR: Datetime invalid\n");
            }

            // clk_sys is >2000x faster than clk_rtc, so datetime is not updated immediately when rtc_get_datetime() is called.
            // tbe delay is up to 3 RTC clock cycles (which is 64us with the default clock settings)
            sleep_us(64);
            updateTime = false;
        }

        QMI8658_read_xyz(acc, gyro, &tim_count);
        // float bat = (*battery.Base.Read)(&battery.Base);
        // printf("Battery: %f\n", bat);
// #define GYRMAX 300.0f
// #define ACCMAX 500.0f
        // theme_bg_dynamic_mode seems to be a mode that requires greater force to register, such 
        // as when in full sleep mode.
        // if (theme_bg_dynamic_mode == 1)
        // {
        //     if ((gyro[0] > -ACCMAX && gyro[0] < ACCMAX) && (gyro[1] > -ACCMAX && gyro[1] < ACCMAX) && (gyro[2] > -ACCMAX && gyro[2] < ACCMAX))
        //     {
        //         no_moveshake = true;
        //     }
        // }
        // else
        // {
        // bool no_moveshake = false;
        // if ((acc[0] > -GYRMAX && acc[0] < GYRMAX) && (acc[1] > -GYRMAX && acc[1] < GYRMAX))
        // {
        //     no_moveshake = true;
        // }
        // }

        uint32_t currentTime = time_us_32();
        if (acc[2] >= 0.0f)
        {
            lastMovementTime = currentTime; 
            if (lowPowerMode)
            {
                (*lcdScreen.Base.SetBacklightPercentage)(&lcdScreen.Base, 90);
                lowPowerMode = true;
            }
        }
        else if (currentTime - lastMovementTime > 5000000)
        {
            // 5 secs. ellapsed without movement.
            (*lcdScreen.Base.SetBacklightPercentage)(&lcdScreen.Base, 5);
            lowPowerMode = true;
        }

        (*app.Lifecycle.Loop)(&app.Resources);
        sleep_ms(100);
    }
}