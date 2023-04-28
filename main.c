#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "PicoBattery.h"
#include "GC9A01A.h"
#include "ByteHelper.h"
#include "Color.h"
#include "ScreenFramebuffer.h"
#include "TextureFontPainter.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

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

    uint16_t image[100 * 100];
    Texture texture = 
    {
        .PixelData = image,
        .Width = 100,
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

    ClearTexture(&texture, BLACK);
    (*fontPainter->SetCursorPosition)(fontPainter, (Point) { .X = 0, .Y = 0 });
    (*fontPainter->DrawString)(fontPainter, "Hello, world!");

    unsigned int screenWidth = (*screen->GetWidth)(screen);
    unsigned int screenHeight = (*screen->GetHeight)(screen);
    unsigned int centerX = screenWidth / 2;
    unsigned int centerY = screenHeight / 2;
    DrawTexture(
        screen, 
        &texture, 
        centerX - (texture.Width / 2),
        centerY - (texture.Height / 2));

    while (true) { }
}