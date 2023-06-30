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
#include "hardware/flash.h"

extern _Colors Colors;

#define CMD_NONE 0x0
#define CMD_SET_TIME 0x1
#define CMD_PLUS 0x2
#define CMD_MINUS 0x3
#define CMD_SELECT 0x4

#define SAVE_STATE_CONTENT_SIZE (FLASH_PAGE_SIZE - 5)
typedef struct SaveState
{
    char Name[5];
    uint8_t Content[SAVE_STATE_CONTENT_SIZE];
}
SaveState;

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
        }
    }

    commandState->PendingCommand = CMD_NONE;
}

static void Core1Main()
{
    multicore_fifo_push_blocking(CORE_READY_FLAG);
    if (multicore_fifo_pop_blocking() != CORE_READY_FLAG)
    {
        return;
    }

    PicoDateTimeProvider_Init(&dateTimeProvider);

    PicoTimer timer;
    PicoTimer_Init(&timer);

    LcdScreen lcdScreen = { };
    LcdScreen_Init(&timer.Base, ScanDirection_Horizontal, &lcdScreen);

    PicoPowerManager powerManager;
    PicoPowerManager_Init(&lcdScreen, &powerManager);

    ColorConverter colorConverter;
    PicoColorConverter_Init(&colorConverter);

    App app;
    App_Init(&lcdScreen.Base, &timer.Base, &dateTimeProvider.Base, &powerManager.Base, &colorConverter, &app);

    QMI8658_init(i2c1);
    uint32_t lastMovementTime = time_us_32(); 
    bool lowPowerMode = false;
    QMI8658_MotionCoordinates acc = { };
    QMI8658_MotionCoordinates gyro = { };
    unsigned int timeCounter = 0;
    CommandState commandState = { .PendingCommand = CMD_NONE };
    while(1)
    {
        if (multicore_fifo_rvalid())
        {
            HandleMessage(&commandState, &app.Input.Base);
        }

        QMI8658_read_xyz(&acc, &gyro, &timeCounter);

        // printf("ACC (%f, %f, %f) GYR (%f, %f, %f)\n", acc.X, acc.Y, acc.Z, gyro.X, gyro.Y, gyro.Z);

        uint32_t currentTime = time_us_32();
        if (acc.Y < -500) // Left wrist facing up.
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

static void HandleCommand(char command[4], bool* cancel)
{
    if (!strcmp("TIM", command))
    {
        printf("Set time\n");
        multicore_fifo_push_blocking(CMD_SET_TIME);

        char unixTimeBuffer[11] = { };
        ReadBuffer(unixTimeBuffer, 10, cancel);
        multicore_fifo_push_blocking((int)atoi(unixTimeBuffer));
    }
    else if (!strcmp("PLS", command))
    {
        printf("Plus\n");
        multicore_fifo_push_blocking(CMD_PLUS);
    }
    else if (!strcmp("MIN", command))
    {
        printf("Minus\n");
        multicore_fifo_push_blocking(CMD_MINUS);
    }
    else if (!strcmp("SEL", command))
    {
        printf("Select\n");
        multicore_fifo_push_blocking(CMD_SELECT);
    }
}

int main(void)
{
    stdio_init_all();
    multicore_launch_core1(Core1Main);

    // Wait for the second core to finish startup.
    if (multicore_fifo_pop_blocking() != CORE_READY_FLAG)
    {
        return 1;
    }

    multicore_fifo_push_blocking(CORE_READY_FLAG);

    while(1)
    {
        bool cancel = false;
        char command[4] = { };
        ReadBuffer(command, 3, &cancel);

        HandleCommand(command, &cancel);
    }
}