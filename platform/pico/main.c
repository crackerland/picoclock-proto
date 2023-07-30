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
#include "PicoColorConverter.h"
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
#include "pico/multicore.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/watchdog.h"

#define MULTICORE 

extern _Colors Colors;

#define CMD_NONE 0x0
#define CMD_SET_TIME 0x1
#define CMD_PLUS 0x2
#define CMD_MINUS 0x3
#define CMD_SELECT 0x4
#define CMD_SLEEP 0x5
#define CMD_RESET 0x6
#define CMD_WAKE 0x7

#define CMD_TEXT_SET_TIME "TIM"
#define CMD_TEXT_PLUS "PLS"
#define CMD_TEXT_MINUS "MIN"
#define CMD_TEXT_SELECT "SEL"
#define CMD_TEXT_SLEEP "SLP"
#define CMD_TEXT_WAKE "WAK"
#define CMD_TEXT_RESET "RST"

typedef struct CommandState
{
    uint8_t PendingCommand;
}
CommandState;

static void SetUpGpioPin(uint16_t pin, bool direction)
{
    gpio_init(pin);
    gpio_set_dir(pin, direction);
}

static PicoDateTimeProvider dateTimeProvider;
static datetime_t pendingTime = { };
static bool updateTime = false;

static void SetTime(uint32_t time, datetime_t* out)
{
    time -= 60 * 60 * 4; // Offset UTC - 4 hours to make EST.
    time_t nowBuffer = (time_t)time;
    struct tm* parsed = localtime(&nowBuffer);

    out->year = parsed->tm_year + 1900; // `tm_year` is the number of years since 1900.
    out->month = parsed->tm_mon;
    out->day = parsed->tm_mday;
    out->dotw = parsed->tm_wday;
    out->hour = parsed->tm_hour;
    out->min = parsed->tm_min;
    out->sec = parsed->tm_sec;
}

#define CORE_READY_FLAG 0xDEADBEEF

static inline void HandleMessage(CommandState* commandState, UserInput* input)
{
    uint32_t message = multicore_fifo_pop_blocking();
    if (commandState->PendingCommand == CMD_SET_TIME)
    {
        SetTime(message, &pendingTime);
        if (!rtc_set_datetime(&pendingTime))
        {
            printf("ERROR: Datetime invalid\n");
        }

        // clk_sys is >2000x faster than clk_rtc, so datetime is not updated immediately when rtc_get_datetime() is called.
        // tbe delay is up to 3 RTC clock cycles (which is 64us with the default clock settings)
        sleep_us(64);
        updateTime = false;
    }
    else
    {
        switch (message)
        {
            case CMD_SET_TIME:
                // Set time requires an argument. Set the pending command and wait for the next message.
                commandState->PendingCommand = CMD_SET_TIME;
                return;

            // The remaining commands are run immediately with no argument.
            case CMD_PLUS:
                (*input->Plus)(input);
                break;

            case CMD_MINUS:
                (*input->Minus)(input);
                break;

            case CMD_SELECT:
                (*input->Select)(input);
                break;

            case CMD_SLEEP:
                (*input->Sleep)(input);
                break;

            case CMD_WAKE:
                (*input->Wake)(input);
                break;

            case CMD_RESET:
                (*input->Reset)(input);
                break;
        }
    }

    commandState->PendingCommand = CMD_NONE;
}

static void ReadBuffer(char* buffer, size_t length, bool* cancel)
{
    unsigned int next = 0;
    while (next < length)
    {
        if (*cancel)
        {
            return;
        }

        int c = PICO_ERROR_TIMEOUT;
        if ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT)
        {
            buffer[next++] = (char)c;
        }
    }
}

#ifndef MULTICORE
static uint32_t pendingMessage = 0;
static bool messagePending = false;
#endif

static inline void PushMessage(uint32_t message)
{
#ifdef MULTICORE
    multicore_fifo_push_blocking(message);
#else
    pendingMessage = message; 
    messagePending = true;
#endif
}

static inline bool MessagePending()
{
#ifdef MULTICORE
    return multicore_fifo_rvalid();
#else
    return messagePending;
#endif
}

static void HandleCommand(char command[4], bool* cancel)
{
    if (!strcmp(CMD_TEXT_SET_TIME, command))
    {
        printf("Set time\n");
        PushMessage(CMD_SET_TIME);

        char unixTimeBuffer[11] = { };
        ReadBuffer(unixTimeBuffer, 10, cancel);
        PushMessage((int)atoi(unixTimeBuffer));
    }
    else if (!strcmp(CMD_TEXT_PLUS, command))
    {
        printf("Plus\n");
        PushMessage(CMD_PLUS);
    }
    else if (!strcmp(CMD_TEXT_MINUS, command))
    {
        printf("Minus\n");
        PushMessage(CMD_MINUS);
    }
    else if (!strcmp(CMD_TEXT_SELECT, command))
    {
        printf("Select\n");
        PushMessage(CMD_SELECT);
    }
    else if (!strcmp(CMD_TEXT_SLEEP, command))
    {
        printf("Sleep\n");
        PushMessage(CMD_SLEEP);
    }
    else if (!strcmp(CMD_TEXT_WAKE, command))
    {
        printf("Wake\n");
        PushMessage(CMD_WAKE);
    }
    else if (!strcmp(CMD_TEXT_RESET, command))
    {
        printf("Reset\n");
        PushMessage(CMD_RESET);
    }
}

static void PollMessage(CommandState* commandState, UserInput* input)
{
#ifndef MULTICORE
    // bool cancel = false;
    // char command[4] = { };
    // ReadBuffer(command, 3, &cancel);
    // HandleCommand(command, &cancel);
#endif

    if (MessagePending())
    {
        HandleMessage(commandState, input);
    }
}

static inline void AppMain()
{
    PicoDateTimeProvider_Init(&dateTimeProvider);

    PicoTimer timer;
    PicoTimer_Init(&timer);

    LcdScreen lcdScreen = { };
    LcdScreen_Init(&timer.Base, ScanDirection_Horizontal, &lcdScreen);

    DeferredTaskScheduler scheduler;
    DeferredTaskScheduler_Init(&scheduler);

    Qmi8658 module;
    Qmi8658_Init(i2c1, &scheduler, &module);

    MotionDevice accelerometer;
    MotionDevice gyro;
    (*module.ConfigureSensors)(
        &module, 
        &(Qmi8658AccelerometerConfig)
        {
            .Enabled = true,
            .Range = QMI8658AccRange_8g,
            .Odr = QMI8658AccOdr_1000Hz,
            .LowPassFilterEnabled = false,
            .SelfTestEnabled = false
        },
        &(Qmi8658GyroscopeConfig)
        {
            .Enabled = true,
            .Range = QMI8658GyrRange_512dps,
            .Odr = QMI8658GyrOdr_1000Hz,
            .LowPassFilterEnabled = false,
            .SelfTestEnabled = false
        },
        &accelerometer,
        &gyro);

    // MotionDevice attitudeEngine;
    // (*module.ConfigureAttitudeEngine)(
    //     &module, 
    //     true, 
    //     QMI8658AeOdr_32Hz,
    //     &attitudeEngine);

    AppPreferences preferences = 
    {
        .SleepTimeoutMillis = 60 * 1000, // 1 minute.
        .DimTimeoutMillis = 7 * 1000, // 7 seconds.
        .FullBrightness = 90,
        .DimBrightness = 5
    };

    PicoPowerManager powerManager;
    PicoPowerManager_Init(&lcdScreen, &module, &gyro, &preferences, &powerManager);
    // PicoPowerManager_Init(&lcdScreen, &module, &attitudeEngine, &powerManager);

    ColorConverter colorConverter;
    PicoColorConverter_Init(&colorConverter);

    App app;
    App_Init(&lcdScreen.Base, &timer.Base, &dateTimeProvider.Base, &powerManager.Base, &colorConverter, &app);

    CommandState commandState = { .PendingCommand = CMD_NONE };
    while(1)
    {
        DeferredTask* task = NULL;
        while ((task = (*scheduler.Poll)(&scheduler)))
        {
            free(task);
        }
        
        PollMessage(&commandState, &app.Input.Base);
        (*powerManager.Update)(&powerManager);
        (*app.Lifecycle.Loop)(&app.Resources);
        sleep_ms(100);
    }
}

static void Core1Main()
{
    multicore_fifo_push_blocking(CORE_READY_FLAG);
    if (multicore_fifo_pop_blocking() != CORE_READY_FLAG)
    {
        return;
    }

    AppMain();
}

int main(void)
{
    stdio_init_all();

    // if (watchdog_caused_reboot()) 
    // {
    //     // Something stalled and caused a reboot.
    //     printf("Rebooted by watchdog.\n");
    // }

    // watchdog_enable(1000, true);

#ifdef MULTICORE
    multicore_launch_core1(Core1Main);

    // Wait for the second core to finish startup.
    if (multicore_fifo_pop_blocking() != CORE_READY_FLAG)
    {
        return 1;
    }

    multicore_fifo_push_blocking(CORE_READY_FLAG);

    while(1)
    {
        // watchdog_update();

        bool cancel = false;
        char command[4] = { };
        ReadBuffer(command, 3, &cancel);
        HandleCommand(command, &cancel);
        sleep_ms(250);
    }
#else
    AppMain();
#endif
}