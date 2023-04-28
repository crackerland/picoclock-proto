#include "LcdScreen.h"
#include <string.h>
#include <math.h>
#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "ByteHelper.h"

#define SPI_PORT spi1
#define LCD_DC_PIN 8
#define LCD_CS_PIN 9
#define LCD_CLK_PIN 10
#define LCD_MOSI_PIN 11
#define LCD_RST_PIN 12
#define LCD_BL_PIN 25

static void DEV_GPIO_Mode(uint16_t pin, bool direction)
{
    gpio_init(pin);
    gpio_set_dir(pin, direction);
}

static void DEV_Digital_Write(uint16_t Pin, uint8_t Value)
{
    gpio_put(Pin, Value);
}

static uint8_t DEV_Digital_Read(uint16_t Pin)
{
    return gpio_get(Pin);
}

static void DEV_Delay_ms(uint32_t xms)
{
    sleep_ms(xms);
}

static void DEV_SPI_WriteByte(uint8_t Value)
{
    spi_write_blocking(SPI_PORT, &Value, 1);
}

static void DEV_SPI_Write_nByte(uint8_t pData[], uint32_t Len)
{
    spi_write_blocking(SPI_PORT, pData, Len);
}

static void StartDraw()
{
    DEV_Digital_Write(LCD_DC_PIN, 1);
}

static void LCD_1IN28_Reset()
{
    DEV_Digital_Write(LCD_RST_PIN, 1);
    DEV_Delay_ms(100);
    DEV_Digital_Write(LCD_RST_PIN, 0);
    DEV_Delay_ms(100);
    DEV_Digital_Write(LCD_RST_PIN, 1);
	DEV_Digital_Write(LCD_CS_PIN, 0);
    DEV_Delay_ms(100);
}

static void LCD_1IN28_SendCommand(uint8_t Reg)
{
    DEV_Digital_Write(LCD_DC_PIN, 0);
    DEV_SPI_WriteByte(Reg);
}

static void LCD_1IN28_SendData_8Bit(uint8_t Data)
{
	StartDraw();
    DEV_SPI_WriteByte(Data);
}

static void LCD_1IN28_SendData_16Bit(uint16_t Data)
{
	StartDraw();
    DEV_SPI_WriteByte(Data >> 8);
    DEV_SPI_WriteByte(Data);
}

static void LCD_1IN28_InitReg(void)
{
    LCD_1IN28_SendCommand(0xEF);
	LCD_1IN28_SendCommand(0xEB);
	LCD_1IN28_SendData_8Bit(0x14); 
	
    LCD_1IN28_SendCommand(0xFE);			 
	LCD_1IN28_SendCommand(0xEF); 

	LCD_1IN28_SendCommand(0xEB);	
	LCD_1IN28_SendData_8Bit(0x14); 

	LCD_1IN28_SendCommand(0x84);			
	LCD_1IN28_SendData_8Bit(0x40); 

	LCD_1IN28_SendCommand(0x85);			
	LCD_1IN28_SendData_8Bit(0xFF); 

	LCD_1IN28_SendCommand(0x86);			
	LCD_1IN28_SendData_8Bit(0xFF); 

	LCD_1IN28_SendCommand(0x87);			
	LCD_1IN28_SendData_8Bit(0xFF);

	LCD_1IN28_SendCommand(0x88);			
	LCD_1IN28_SendData_8Bit(0x0A);

	LCD_1IN28_SendCommand(0x89);			
	LCD_1IN28_SendData_8Bit(0x21); 

	LCD_1IN28_SendCommand(0x8A);			
	LCD_1IN28_SendData_8Bit(0x00); 

	LCD_1IN28_SendCommand(0x8B);			
	LCD_1IN28_SendData_8Bit(0x80); 

	LCD_1IN28_SendCommand(0x8C);			
	LCD_1IN28_SendData_8Bit(0x01); 

	LCD_1IN28_SendCommand(0x8D);			
	LCD_1IN28_SendData_8Bit(0x01); 

	LCD_1IN28_SendCommand(0x8E);			
	LCD_1IN28_SendData_8Bit(0xFF); 

	LCD_1IN28_SendCommand(0x8F);			
	LCD_1IN28_SendData_8Bit(0xFF); 


	LCD_1IN28_SendCommand(0xB6);
	LCD_1IN28_SendData_8Bit(0x00);
	LCD_1IN28_SendData_8Bit(0x20);

	LCD_1IN28_SendCommand(0x36);
	LCD_1IN28_SendData_8Bit(0x08);//Set as vertical screen

	LCD_1IN28_SendCommand(0x3A);			
	LCD_1IN28_SendData_8Bit(0x05); 


	LCD_1IN28_SendCommand(0x90);			
	LCD_1IN28_SendData_8Bit(0x08);
	LCD_1IN28_SendData_8Bit(0x08);
	LCD_1IN28_SendData_8Bit(0x08);
	LCD_1IN28_SendData_8Bit(0x08); 

	LCD_1IN28_SendCommand(0xBD);			
	LCD_1IN28_SendData_8Bit(0x06);
	
	LCD_1IN28_SendCommand(0xBC);			
	LCD_1IN28_SendData_8Bit(0x00);	

	LCD_1IN28_SendCommand(0xFF);			
	LCD_1IN28_SendData_8Bit(0x60);
	LCD_1IN28_SendData_8Bit(0x01);
	LCD_1IN28_SendData_8Bit(0x04);

	LCD_1IN28_SendCommand(0xC3);			
	LCD_1IN28_SendData_8Bit(0x13);
	LCD_1IN28_SendCommand(0xC4);			
	LCD_1IN28_SendData_8Bit(0x13);

	LCD_1IN28_SendCommand(0xC9);			
	LCD_1IN28_SendData_8Bit(0x22);

	LCD_1IN28_SendCommand(0xBE);			
	LCD_1IN28_SendData_8Bit(0x11); 

	LCD_1IN28_SendCommand(0xE1);			
	LCD_1IN28_SendData_8Bit(0x10);
	LCD_1IN28_SendData_8Bit(0x0E);

	LCD_1IN28_SendCommand(0xDF);			
	LCD_1IN28_SendData_8Bit(0x21);
	LCD_1IN28_SendData_8Bit(0x0c);
	LCD_1IN28_SendData_8Bit(0x02);

	LCD_1IN28_SendCommand(0xF0);   
	LCD_1IN28_SendData_8Bit(0x45);
	LCD_1IN28_SendData_8Bit(0x09);
	LCD_1IN28_SendData_8Bit(0x08);
	LCD_1IN28_SendData_8Bit(0x08);
	LCD_1IN28_SendData_8Bit(0x26);
 	LCD_1IN28_SendData_8Bit(0x2A);

 	LCD_1IN28_SendCommand(0xF1);    
 	LCD_1IN28_SendData_8Bit(0x43);
 	LCD_1IN28_SendData_8Bit(0x70);
 	LCD_1IN28_SendData_8Bit(0x72);
 	LCD_1IN28_SendData_8Bit(0x36);
 	LCD_1IN28_SendData_8Bit(0x37);  
 	LCD_1IN28_SendData_8Bit(0x6F);


 	LCD_1IN28_SendCommand(0xF2);   
 	LCD_1IN28_SendData_8Bit(0x45);
 	LCD_1IN28_SendData_8Bit(0x09);
 	LCD_1IN28_SendData_8Bit(0x08);
 	LCD_1IN28_SendData_8Bit(0x08);
 	LCD_1IN28_SendData_8Bit(0x26);
 	LCD_1IN28_SendData_8Bit(0x2A);

 	LCD_1IN28_SendCommand(0xF3);   
 	LCD_1IN28_SendData_8Bit(0x43);
 	LCD_1IN28_SendData_8Bit(0x70);
 	LCD_1IN28_SendData_8Bit(0x72);
 	LCD_1IN28_SendData_8Bit(0x36);
 	LCD_1IN28_SendData_8Bit(0x37); 
 	LCD_1IN28_SendData_8Bit(0x6F);

	LCD_1IN28_SendCommand(0xED);	
	LCD_1IN28_SendData_8Bit(0x1B); 
	LCD_1IN28_SendData_8Bit(0x0B); 

	LCD_1IN28_SendCommand(0xAE);			
	LCD_1IN28_SendData_8Bit(0x77);
	
	LCD_1IN28_SendCommand(0xCD);			
	LCD_1IN28_SendData_8Bit(0x63);		


	LCD_1IN28_SendCommand(0x70);			
	LCD_1IN28_SendData_8Bit(0x07);
	LCD_1IN28_SendData_8Bit(0x07);
	LCD_1IN28_SendData_8Bit(0x04);
	LCD_1IN28_SendData_8Bit(0x0E); 
	LCD_1IN28_SendData_8Bit(0x0F); 
	LCD_1IN28_SendData_8Bit(0x09);
	LCD_1IN28_SendData_8Bit(0x07);
	LCD_1IN28_SendData_8Bit(0x08);
	LCD_1IN28_SendData_8Bit(0x03);

	LCD_1IN28_SendCommand(0xE8);			
	LCD_1IN28_SendData_8Bit(0x34);

	LCD_1IN28_SendCommand(0x62);			
	LCD_1IN28_SendData_8Bit(0x18);
	LCD_1IN28_SendData_8Bit(0x0D);
	LCD_1IN28_SendData_8Bit(0x71);
	LCD_1IN28_SendData_8Bit(0xED);
	LCD_1IN28_SendData_8Bit(0x70); 
	LCD_1IN28_SendData_8Bit(0x70);
	LCD_1IN28_SendData_8Bit(0x18);
	LCD_1IN28_SendData_8Bit(0x0F);
	LCD_1IN28_SendData_8Bit(0x71);
	LCD_1IN28_SendData_8Bit(0xEF);
	LCD_1IN28_SendData_8Bit(0x70); 
	LCD_1IN28_SendData_8Bit(0x70);

	LCD_1IN28_SendCommand(0x63);			
	LCD_1IN28_SendData_8Bit(0x18);
	LCD_1IN28_SendData_8Bit(0x11);
	LCD_1IN28_SendData_8Bit(0x71);
	LCD_1IN28_SendData_8Bit(0xF1);
	LCD_1IN28_SendData_8Bit(0x70); 
	LCD_1IN28_SendData_8Bit(0x70);
	LCD_1IN28_SendData_8Bit(0x18);
	LCD_1IN28_SendData_8Bit(0x13);
	LCD_1IN28_SendData_8Bit(0x71);
	LCD_1IN28_SendData_8Bit(0xF3);
	LCD_1IN28_SendData_8Bit(0x70); 
	LCD_1IN28_SendData_8Bit(0x70);

	LCD_1IN28_SendCommand(0x64);			
	LCD_1IN28_SendData_8Bit(0x28);
	LCD_1IN28_SendData_8Bit(0x29);
	LCD_1IN28_SendData_8Bit(0xF1);
	LCD_1IN28_SendData_8Bit(0x01);
	LCD_1IN28_SendData_8Bit(0xF1);
	LCD_1IN28_SendData_8Bit(0x00);
	LCD_1IN28_SendData_8Bit(0x07);

	LCD_1IN28_SendCommand(0x66);			
	LCD_1IN28_SendData_8Bit(0x3C);
	LCD_1IN28_SendData_8Bit(0x00);
	LCD_1IN28_SendData_8Bit(0xCD);
	LCD_1IN28_SendData_8Bit(0x67);
	LCD_1IN28_SendData_8Bit(0x45);
	LCD_1IN28_SendData_8Bit(0x45);
	LCD_1IN28_SendData_8Bit(0x10);
	LCD_1IN28_SendData_8Bit(0x00);
	LCD_1IN28_SendData_8Bit(0x00);
	LCD_1IN28_SendData_8Bit(0x00);

	LCD_1IN28_SendCommand(0x67);			
	LCD_1IN28_SendData_8Bit(0x00);
	LCD_1IN28_SendData_8Bit(0x3C);
	LCD_1IN28_SendData_8Bit(0x00);
	LCD_1IN28_SendData_8Bit(0x00);
	LCD_1IN28_SendData_8Bit(0x00);
	LCD_1IN28_SendData_8Bit(0x01);
	LCD_1IN28_SendData_8Bit(0x54);
	LCD_1IN28_SendData_8Bit(0x10);
	LCD_1IN28_SendData_8Bit(0x32);
	LCD_1IN28_SendData_8Bit(0x98);

	LCD_1IN28_SendCommand(0x74);			
	LCD_1IN28_SendData_8Bit(0x10);	
	LCD_1IN28_SendData_8Bit(0x85);	
	LCD_1IN28_SendData_8Bit(0x80);
	LCD_1IN28_SendData_8Bit(0x00); 
	LCD_1IN28_SendData_8Bit(0x00); 
	LCD_1IN28_SendData_8Bit(0x4E);
	LCD_1IN28_SendData_8Bit(0x00);					
	
    LCD_1IN28_SendCommand(0x98);			
	LCD_1IN28_SendData_8Bit(0x3e);
	LCD_1IN28_SendData_8Bit(0x07);

	LCD_1IN28_SendCommand(0x35);	
	LCD_1IN28_SendCommand(0x21);

	LCD_1IN28_SendCommand(0x11);
	DEV_Delay_ms(120);
	LCD_1IN28_SendCommand(0x29);
	DEV_Delay_ms(20);
}

// Sets the resolution and scanning method of the screen.
static void LCD_1IN28_SetAttributes(LcdScreen* screen, ScanDirection scanDir)
{
    //Get the screen scan direction
    screen->Attributes.ScanDir = scanDir;
    uint8_t MemoryAccessReg = 0x08;

    //Get GRAM and LCD width and height
    if (scanDir == ScanDirection_Horizontal) 
	{
        screen->Attributes.Height = LCD_1IN28_HEIGHT;
        screen->Attributes.Width = LCD_1IN28_WIDTH;
        MemoryAccessReg = 0Xc8;
    } 
	else 
	{
        screen->Attributes.Height = LCD_1IN28_WIDTH;
        screen->Attributes.Width = LCD_1IN28_HEIGHT;
        MemoryAccessReg = 0X68;
    }

    // Set the read / write scan direction of the frame memory
    LCD_1IN28_SendCommand(0x36); //MX, MY, RGB mode
	LCD_1IN28_SendData_8Bit(MemoryAccessReg);	//0x08 set RGB
}

static void SetViewport(Screen* screen, uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd)
{
    // Set the X coordinates
    LCD_1IN28_SendCommand(0x2A);
    LCD_1IN28_SendData_8Bit(0x00);
    LCD_1IN28_SendData_8Bit(xStart);
	LCD_1IN28_SendData_8Bit((xEnd-1)>>8);
    LCD_1IN28_SendData_8Bit(xEnd-1);

    // Set the Y coordinates
    LCD_1IN28_SendCommand(0x2B);
    LCD_1IN28_SendData_8Bit(0x00);
	LCD_1IN28_SendData_8Bit(yStart);
	LCD_1IN28_SendData_8Bit((xEnd-1)>>8);
    LCD_1IN28_SendData_8Bit(yEnd-1);

    LCD_1IN28_SendCommand(0X2C);
}

static void Clear(Screen* screen, uint16_t color)
{
    uint16_t image[LCD_1IN28_WIDTH * LCD_1IN28_HEIGHT];
    
	color = ByteHelper_Reverse16(color);
   
    for (uint16_t j = 0; j < LCD_1IN28_HEIGHT*LCD_1IN28_WIDTH; j++) 
    {
        image[j] = color;
    }
    
    SetViewport(screen, 0, 0, LCD_1IN28_WIDTH, LCD_1IN28_HEIGHT);
	StartDraw();
    for(uint16_t j = 0; j < LCD_1IN28_HEIGHT; j++)
    {
        DEV_SPI_Write_nByte((uint8_t *)&image[j*LCD_1IN28_WIDTH], LCD_1IN28_WIDTH*2);
    }
}

static void DrawBuffer(Screen* screen, uint16_t *image)
{
    SetViewport(screen, 0, 0, LCD_1IN28_WIDTH, LCD_1IN28_HEIGHT);
	StartDraw();
    for (uint16_t i = 0; i < LCD_1IN28_HEIGHT; i++) 
    {
        DEV_SPI_Write_nByte(
            (uint8_t*)&image[i * LCD_1IN28_WIDTH], 
            LCD_1IN28_WIDTH * 2);
    }
}

static void DisplayWindows(
    Screen* screen, 
    unsigned int xStart, 
    unsigned int yStart, 
    unsigned int xEnd, 
    unsigned int yEnd, 
    uint8_t *image,
	size_t pixelSize)
{
    SetViewport(screen, xStart, yStart, xEnd, yEnd);
	StartDraw();

	unsigned int width = xEnd - xStart;
	unsigned int height = yEnd - yStart;
	DEV_SPI_Write_nByte(image, pixelSize * width * height);
}

static void DrawPoint(Screen* screen, uint16_t x, uint16_t y, uint16_t color)
{
    SetViewport(screen, x, y, x, y);
    LCD_1IN28_SendData_16Bit(color);
}

static void SetBacklightPercentage(Screen* screen, uint8_t percentage)
{
    LcdScreen* this = (LcdScreen*)screen;

    pwm_set_chan_level(
		this->BacklightPwnSliceNumber, 
		PWM_CHAN_B, 
		MIN(MAX(percentage, 0), 100)); 
}

static unsigned int GetWidth(Screen* screen)
{
	return LCD_1IN28_WIDTH; 
}

static unsigned int GetHeight(Screen* screen)
{
	return LCD_1IN28_HEIGHT; 
}

void LcdScreen_Init(LcdScreen* out, ScanDirection scanDirection)
{
    LcdScreen screen = 
    {
        .Base =
        {
			.SetViewport = SetViewport,
            .Clear = Clear,
            .DrawBuffer = DrawBuffer, 
            .DisplayWindows = DisplayWindows,
            .DrawPoint = DrawPoint,
			.SetBacklightPercentage = SetBacklightPercentage,
			.GetHeight = GetHeight,
			.GetWidth = GetWidth
        }
    };

    // LCD GPIO config
    DEV_GPIO_Mode(LCD_RST_PIN, 1);
    DEV_GPIO_Mode(LCD_DC_PIN, 1);
    DEV_GPIO_Mode(LCD_CS_PIN, 1);
    DEV_GPIO_Mode(LCD_BL_PIN, 1);

    gpio_put(LCD_CS_PIN, 1);
    gpio_put(LCD_DC_PIN, 0);
    gpio_put(LCD_BL_PIN, 1);

    // Backlight PWM Config
    gpio_set_function(LCD_BL_PIN, GPIO_FUNC_PWM);
    uint pwmSliceNum = pwm_gpio_to_slice_num(LCD_BL_PIN);
    pwm_set_wrap(pwmSliceNum, 100);
    pwm_set_chan_level(pwmSliceNum, PWM_CHAN_B, 0);
    pwm_set_clkdiv(pwmSliceNum, 50);
    pwm_set_enabled(pwmSliceNum, true);
	screen.BacklightPwnSliceNumber = pwmSliceNum;

    // SPI Config
    spi_init(SPI_PORT, 40000 * 1000);
    gpio_set_function(LCD_CLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(LCD_MOSI_PIN, GPIO_FUNC_SPI);

    //Hardware reset
    LCD_1IN28_Reset();

    //Set the resolution and scanning method of the screen
    LCD_1IN28_SetAttributes(&screen, scanDirection);
    
    //Set the initialization register
    LCD_1IN28_InitReg();

    memcpy(out, &screen, sizeof(LcdScreen));
}