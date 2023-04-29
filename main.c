#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "PicoBattery.h"
#include "GC9A01A.h"
#include "ByteHelper.h"
#include "Color.h"
#include "DateTimeFormatter.h"
#include "ScreenFramebuffer.h"
#include "TextureFontPainter.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/rtc.h"
#include "hardware/flash.h"
#include "pico/util/datetime.h"

extern _Colors Colors;

#define I2C_PORT i2c1
#define DEV_SDA_PIN     (6)
#define DEV_SCL_PIN     (7)

#define WHITE          0xFFFF
#define BLACK          0x0000
#define BLUE           0x001F
#define BRED           0XF81F
#define GRED           0XFFE0
#define ORANGE         0XFE20
#define GBLUE          0X07FF
#define RED            0xF800
#define MAGENTA        0xF81F
#define GREEN          0x07E0
#define CYAN           0x7FFF
#define YELLOW         0xFFE0
#define BROWN          0XBC40
#define BRRED          0XFC07
#define GRAY           0X8430
#define LGRAY          0X8551
#define NBLACK         0x0821
#define NWHITE         0xFFFE

#define SAVE_STATE_CONTENT_SIZE (FLASH_PAGE_SIZE - 5)
typedef struct SaveState
{
    char Name[5];
    uint8_t Content[SAVE_STATE_CONTENT_SIZE];
}
SaveState;

typedef struct Texture
{
    uint16_t* PixelData;
    unsigned int Width;
    unsigned int Height;
}
Texture;

static void DrawTexture(Screen* screen, Texture* texture, unsigned int x, unsigned int y)
{
    (*screen->SetViewport)(
        screen, 
        x, 
        y, 
        x + texture->Width, 
        y + texture->Height);

    (*screen->DrawBuffer)(
        screen, 
        (uint8_t*)texture->PixelData,
        sizeof(uint16_t) * texture->Width * texture->Height);
}

static void ClearTexture(Texture* texture, uint16_t color)
{
    color = ByteHelper_Reverse16(color);
    unsigned int size = texture->Width * texture->Height;
    for (uint16_t i = 0; i < size; i++)
    {
        texture->PixelData[i] = color;
    }
}

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

static void DrawPoint(Color color, Point point, void* payload)
{
    Texture* texture = (Texture*)payload;
    uint16_t pixelColor = Color_ToRgb565(color);
    texture->PixelData[point.Y * texture->Width + point.X] = pixelColor;
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

static Size MeasureString(char* string, FontPainter* fontPainter)
{
    Size totalSize = { };
    int length = strlen(string);
    for (int i = 0; i < length; i++)
    {
        Size glyphSize = (*fontPainter->MeasureGlyph)(fontPainter, string[i]);
        totalSize.Width += glyphSize.Width;
        totalSize.Height = MAX(totalSize.Height, glyphSize.Height);
    }

    return totalSize;
}

int main(void)
{
    stdio_init_all();
    DEV_Module_Init();

    PicoBattery battery = { };
    PicoBattery_Init(&battery);

    PicoBattery bat0 =
    {
        .mode = "GOOD\0",
        .mA = 150.0f, // 150mA
        .load = 4.16f,
        .max = 3.325781f,
        .min = 4.158838f,
        .dif = 0.0f,
        .read = 0.0f
    }; 

    PicoBattery_Init(&bat0);

    LcdScreen lcdScreen = { };
    LcdScreen_Init(&lcdScreen, ScanDirection_Horizontal);
    Screen* screen = &lcdScreen.Base;

    (*screen->Clear)(screen, BLACK);
    (*screen->SetBacklightPercentage)(screen, 50);
    unsigned int screenWidth = (*screen->GetWidth)(screen);
    unsigned int screenHeight = (*screen->GetHeight)(screen);

    uint16_t image[screenWidth * 100];
    Texture texture = 
    {
        .PixelData = image,
        .Width = screenWidth,
        .Height = 100 
    };

    TextureFontPainter textureFontPainter;
    TextureFontPainter_Init(
        DrawPoint, 
        Colors.Green, 
        Colors.Black, 
        texture.Width, 
        texture.Height, 
        &texture, 
        &textureFontPainter);

    FontPainter* fontPainter = &textureFontPainter.Base; 

    unsigned int centerX = screenWidth / 2;
    unsigned int centerY = screenHeight / 2;

    rtc_init();

    // Start on Friday June 5 2020 15:45:00
    datetime_t time = 
    {
        .year  = 2023,
        .month = 4,
        .day   = 28,
        .dotw  = 5, // 0 is Sunday, so 5 is Friday
        .hour  = 20,
        .min   = 7,
        .sec   = 00
    };

    SaveState* savedState = (SaveState*)LoadFlashState();
    if (!strcmp(savedState->Name, "SAVE"))
    {
        // Previous time was saved.
        datetime_t* savedTime = (datetime_t*)savedState->Content;
        memcpy(&time, savedTime, sizeof(datetime_t));
    }

    rtc_set_datetime(&time);

    // clk_sys is >2000x faster than clk_rtc, so datetime is not updated immediately when rtc_get_datetime() is called.
    // tbe delay is up to 3 RTC clock cycles (which is 64us with the default clock settings)
    sleep_us(64);

    char dateString[256] = { };
    char timeString[256] = { };
    while (true) 
    { 
        rtc_get_datetime(&time);

        GetDayAndMonth(&time, dateString);
        GetHoursMinutesSeconds(&time, timeString);

        ClearTexture(&texture, BLACK);
        (*fontPainter->SetCursorPosition)(fontPainter, (Point) { .X = 0, .Y = 0 });
        (*fontPainter->DrawString)(fontPainter, dateString);

        Size dateSize = MeasureString(dateString, fontPainter);
        Size timeSize = MeasureString(timeString, fontPainter);
        Point cursor = (*fontPainter->GetCursorPosition)(fontPainter);
        cursor.Y += dateSize.Height;
        cursor.X = 0;
        (*fontPainter->SetCursorPosition)(fontPainter, cursor);
        (*fontPainter->DrawString)(fontPainter, timeString);

        DrawTexture(screen, &texture, centerX - (dateSize.Width / 2), centerY - dateSize.Height);

        sleep_ms(1000);

        SaveFlashState((uint8_t*)&time, sizeof(datetime_t));
    }
}