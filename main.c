#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "PicoClock.h"
#include "PicoTimer.h"
#include "PicoDateTimeProvider.h"
#include "DefaultDateTimeFormatter.h"
#include "pico/stdlib.h"
#include "PicoBattery.h"
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

#define I2C_PORT i2c1
#define DEV_SDA_PIN     (6)
#define DEV_SCL_PIN     (7)

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

static uint DEV_Module_Init(void)
{
    // I2C Config
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(DEV_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(DEV_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(DEV_SDA_PIN);
    gpio_pull_up(DEV_SCL_PIN);
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

// RX interrupt handler
void on_uart_rx() 
{
    uint8_t buffer[256] = { };
    uint8_t* next = buffer;
    while (uart_is_readable(uart0)) 
    {
        *next++ = uart_getc(uart0);
    }

    datetime_t* time = (datetime_t*)buffer;
    rtc_set_datetime(time);
}

int main(void)
{
    stdio_init_all();
    DEV_Module_Init();

    uart_init(uart0, 115200);
    gpio_set_function(1, GPIO_FUNC_UART);

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
    irq_set_enabled(UART0_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(uart0, true, false);

    LcdScreen lcdScreen = { };
    LcdScreen_Init(&lcdScreen, ScanDirection_Horizontal);

    PicoDateTimeProvider_Init(&dateTimeProvider);

    PicoTimer timer;
    PicoTimer_Init(&timer);

    App_Init(&lcdScreen.Base, &timer.Base, &dateTimeProvider.Base);
}