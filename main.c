#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "PicoBattery.h"
#include "LcdScreen.h"
#include "ByteHelper.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

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
    (*screen->DisplayWindows)(
        screen, 
        x, 
        y, 
        x + texture->Width, 
        y + texture->Height, 
        texture->PixelData,
        sizeof(uint16_t));
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

static void DEV_GPIO_Mode(uint16_t pin, bool direction)
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

    (*screen->Clear)(screen, GREEN);
    (*screen->SetBacklightPercentage)(screen, 60);

    uint16_t image[50 * 50];
    Texture texture = 
    {
        .PixelData = image,
        .Width = 50,
        .Height = 50
    };

    ClearTexture(&texture, RED);

    unsigned int screenWidth = (*screen->GetWidth)(screen);
    unsigned int screenHeight = (*screen->GetHeight)(screen);
    DrawTexture(
        screen, 
        &texture, 
        (screenWidth / 2) - (texture.Width / 2),
        (screenHeight / 2) - (texture.Height / 2));

    while (true) { }
}