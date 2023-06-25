#include "GC9A01A.h"
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

// 6.1 Regulative Command Set (Level 1)
#define CMD_READ_DISPLAY_ID_INFO_2 0x04
#define CMD_READ_DISPLAY_STATUS 0x09
#define CMD_ENTER_SLEEP_MODE 0x10
#define CMD_SLEEP_OUT 0x11
#define CMD_PARTIAL_MODE_ON 0x12
#define CMD_NORMAL_DISPLAY_MODE_ON 0x13
#define CMD_DISPLAY_INVERSION_OFF 0x20
#define CMD_DISPLAY_INVERSION_ON 0x21
#define CMD_DISPLAY_OFF 0x28
#define CMD_DISPLAY_ON 0x29
#define CMD_COLUMN_ADDRESS_SET 0x2A
#define CMD_PAGE_ADDRESS_SET 0x2B
#define CMD_MEMORY_WRITE 0x2C
#define CMD_PARTIAL_AREA 0x30
#define CMD_VERTICAL_SCROLLING_DEFINITION 0x33
#define CMD_TEARING_EFFECT_LINE_OFF 0x34
#define CMD_TEARING_EFFECT_LINE_ON 0x35
#define CMD_MEMORY_ACCESS_CONTROL 0x36
#define CMD_VERTICAL_SCROLLING_START_ADDRESS 0x37
#define CMD_IDLE_MODE_OFF 0x38
#define CMD_IDLE_MODE_ON 0x39 // COLMOD Idle Mode ON - See 6.2.22
#define CMD_PIXEL_FORMAT_SET 0x3A
#define CMD_WRITE_MEMORY_CONTINUE 0x3C
#define CMD_SET_TEAR_SCANLINE 0x44
#define CMD_GET_SCANLINE 0x45
#define CMD_WRITE_DISPLAY_BRIGHTNESS 0x51
#define CMD_WRITE_CTRL_DISPLAY 0x53
#define CMD_READ_ID_1 0xDA
#define CMD_READ_ID_2 0xDB
#define CMD_READ_ID_3 0xDC

// Extended Command Set (Level 2)
#define CMD_RGB_INTERFACE_SIGNAL_CTRL 0xB0
#define CMD_BLANKING_PORCH_CTRL 0xB5
#define CMD_DISPLAY_FUNCTION_CTRL 0xB6
#define CMD_TE_CTRL 0xBA
#define CMD_INTERFACE_CTRL 0xC0

// Inter Command Set (Level 3)
#define CMD_POWER_CRITERION_CTRL 0xC1
#define CMD_VCORE_VOLTAGE_CTRL 0xA7
#define CMD_VREG1A_VOLTAGE_CTRL 0xC3
#define CMD_VREG1B_VOLTAGE_CTRL 0xC4
#define CMD_VREG2A_VOLTAGE_CTRL 0xC9
#define CMD_FRAME_RATE 0xE8
#define CMD_SPI2_DATA_CTRL 0xE9
#define CMD_CHARGE_PUMP_FREQUENT_CTRL 0xEC
#define CMD_INTER_REGISTER_ENABLE_1 0xFE
#define CMD_INTER_REGISTER_ENABLE_2 0xEF
#define CMD_SET_GAMMA_1 0xF0
#define CMD_SET_GAMMA_2 0xF1
#define CMD_SET_GAMMA_3 0xF2
#define CMD_SET_GAMMA_4 0xF3

static void SetUpGpioPin(uint16_t pin, bool direction)
{
    gpio_init(pin);
    gpio_set_dir(pin, direction);
}

static void SpiWriteByte(uint8_t value)
{
    spi_write_blocking(SPI_PORT, &value, 1);
}

static void SpiWriteBytes(uint8_t data[], uint32_t length)
{
    spi_write_blocking(SPI_PORT, data, length);
}

static void StartDraw()
{
    gpio_put(LCD_DC_PIN, 1);
}

static void Reset(LcdScreen* this)
{
    gpio_put(LCD_RST_PIN, 1);
	(*this->Timer->WaitMilliseconds)(this->Timer, 100);
    gpio_put(LCD_RST_PIN, 0);
	(*this->Timer->WaitMilliseconds)(this->Timer, 100);
    gpio_put(LCD_RST_PIN, 1);
	gpio_put(LCD_CS_PIN, 0);
	(*this->Timer->WaitMilliseconds)(this->Timer, 100);
}

static void SendCommand(uint8_t registerAddress)
{
    gpio_put(LCD_DC_PIN, 0);
    SpiWriteByte(registerAddress);
}

static void SendData8(uint8_t Data)
{
	StartDraw();
    SpiWriteByte(Data);
}

static void SendData16(uint16_t Data)
{
	StartDraw();
    SpiWriteByte(Data >> 8);
    SpiWriteByte(Data);
}

// 7  6  5  4  3   2  1 0
// MY MX MV ML BGR MH 0 0
//
// MY: Row Address Order     }
// MX: Column Address Order  } These 3 bits control MCU to memory write/read direction.
// MV: Row/Column Exchange   }
// ML: Vertical Refresh Order LCD vertical refresh direction control.
// BGR: RGB-BGR Order - Color selector switch control (0=RGB color filter panel, 1=BGR color filter panel)
// MH: Horizontal Refresh ORDER LCD horizontal refreshing direction control.
static void ConfigureMemoryAccessControl(
	bool my, 
	bool mx, 
	bool mv, 
	bool verticalRefreshOrder, 
	bool useBgr, 
	bool horizontalRefresh)
{
	SendCommand(CMD_MEMORY_ACCESS_CONTROL);
	SendData8(
		(my ? (1 << 7) : 0)
		| (mx ? (1 << 6) : 0)
		| (mv ? (1 << 5) : 0)
		| (verticalRefreshOrder ? (1 << 4) : 0)
		| (useBgr ? (1 << 3) : 0)
		| (horizontalRefresh ? (1 << 2) : 0));
}

// Sets the resolution and scanning method of the screen.
static void LCD_1IN28_SetAttributes(LcdScreen* screen, ScanDirection scanDir)
{
    //Get the screen scan direction
    screen->Attributes.ScanDir = scanDir;
    uint8_t scanDirRegister;

    //Get GRAM and LCD width and height
    if (scanDir == ScanDirection_Horizontal) 
	{
        screen->Attributes.Height = LCD_1IN28_HEIGHT;
        screen->Attributes.Width = LCD_1IN28_WIDTH;
		ConfigureMemoryAccessControl(true, true, false, false, false, false);

    } 
	else 
	{
        screen->Attributes.Height = LCD_1IN28_WIDTH;
        screen->Attributes.Width = LCD_1IN28_HEIGHT;
		ConfigureMemoryAccessControl(false, true, true, false, false, false);
    }
}

static void SetViewport(Screen* screen, uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd)
{
    // Set the X coordinates
    SendCommand(CMD_COLUMN_ADDRESS_SET);
    SendData8(0x00);
    SendData8(xStart);
	SendData8((xEnd - 1) >> 8);
    SendData8(xEnd - 1);

    // Set the Y coordinates
    SendCommand(CMD_PAGE_ADDRESS_SET);
    SendData8(0x00);
	SendData8(yStart);
	SendData8((xEnd-1)>>8);
    SendData8(yEnd-1);

    SendCommand(CMD_MEMORY_WRITE);
}

static void Clear(Screen* screen, Color color)
{
    uint16_t image[LCD_1IN28_WIDTH * LCD_1IN28_HEIGHT];
	uint16_t pixelColor = Color_ToRgb565(color);
   
    for (uint16_t i = 0; i < LCD_1IN28_HEIGHT * LCD_1IN28_WIDTH; i++) 
    {
        image[i] = pixelColor;
    }
    
    SetViewport(screen, 0, 0, LCD_1IN28_WIDTH, LCD_1IN28_HEIGHT);
	StartDraw();

	uint8_t* bufferRow = (uint8_t*)image;
    for(uint16_t row = 0; row < LCD_1IN28_HEIGHT; row++)
    {
		uint32_t size = LCD_1IN28_WIDTH * sizeof(uint16_t); 
        SpiWriteBytes(bufferRow, size);
		bufferRow += size;
    }
}

static void DrawBuffer(Screen* screen, uint8_t* buffer, size_t size)
{
	StartDraw();
	SpiWriteBytes(buffer, size);
}

static void SetBacklightPercentage(Screen* screen, uint8_t percentage)
{
    LcdScreen* this = (LcdScreen*)screen;

	this->CurrentBacklightPercentage = MIN(MAX(percentage, 0), 100);

    pwm_set_chan_level(
		this->BacklightPwmSliceNumber, 
		PWM_CHAN_B, 
		this->CurrentBacklightPercentage); 
}

static unsigned int GetWidth(Screen* screen)
{
	return LCD_1IN28_WIDTH; 
}

static unsigned int GetHeight(Screen* screen)
{
	return LCD_1IN28_HEIGHT; 
}

static void SetSleep(LcdScreen* screen, bool sleep)
{
    SendCommand(sleep ? CMD_ENTER_SLEEP_MODE : CMD_SLEEP_OUT);

	// Note from docs:
	// "It will be necessary to wait 5 msec before sending next to command, 
	// this is to allow time for the supply voltages and clock circuits to stabilize. 
	// It will be necessary to wait 120 msec after sending Sleep Out command (when in Sleep In Mode) 
	// before Sleep In command can be sent"
	(*screen->Timer->WaitMilliseconds)(screen->Timer, 5);
}

static void SetIdleMode(LcdScreen* screen, bool sleep)
{
    SendCommand(sleep ? CMD_IDLE_MODE_ON : CMD_IDLE_MODE_OFF);
}

static void SetDisplayEnabled(LcdScreen* screen, bool enable)
{
    SendCommand(enable ? CMD_DISPLAY_ON : CMD_DISPLAY_OFF);
}

// reverseGsDirection: Whether to reverse the gate output scan (GS) direction to be high->low. Defaults to low->high.  
// reverseSsDirection: Whether to reverse the source output scan (SS) direction to be high->low. Defaults to low->high.
static void ConfigureDisplayFunctionControl(bool reverseGsDirection, bool reverseSsDirection)
{
	SendCommand(CMD_DISPLAY_FUNCTION_CTRL);
	SendData8(0x00); // (Docs) note: the first parameter must write, but it is not valid.

    // D7 ------------ D0
    // x GS SS x x x x x
	SendData8((reverseGsDirection << 6) | (reverseSsDirection << 5));

    // SM: Sets the gate driver pin arrangement in combination with the GS bit to select the optimal
    // scan mode for the module
    // NL [5:0]: Sets the number of lines to drive the LCD at an interval of 8 lines. The GRAM
    // address mapping is not affected
    // by the number of lines set by NL [5:0]. The number of lines must be the same or more than the
    // number of lines necessary
    // for the size of the liquid crystal panel.
}

static inline void ConfigureSpi2DataControl()
{
    // 2DATA_EN: Set 2_data_line mode in 3-wire/4-wire SPI.
    // 2DATA_MDT[2:0] Set pixel data format in 2_data_line mode.
    //
    // 2DATA_MDT[2:0] | Data Format
    //     000        | 65K color 1pixle/transition
    //     001        | 262K color 1pixle/transition
    //     010        | 262K color 2/3pixle/transition
    //     100        | 4M color 1pixle/transition
    //     110        | 4M color 2/3pixle/transition
    SendCommand(CMD_INTER_REGISTER_ENABLE_2);
	SendCommand(0xEB);
	SendData8(0x14); 
}

static inline void ConfigureUndocumentedCommands()
{
    SendCommand(CMD_INTER_REGISTER_ENABLE_1);			 
	SendCommand(CMD_INTER_REGISTER_ENABLE_2); 

	SendCommand(0xEB);	
	SendData8(0x14); 

	SendCommand(0x84);			
	SendData8(0x40); 

	SendCommand(0x85);			
	SendData8(0xFF); 

	SendCommand(0x86);			
	SendData8(0xFF); 

	SendCommand(0x87);			
	SendData8(0xFF);

	SendCommand(0x88);			
	SendData8(0x0A);

	SendCommand(0x89);			
	SendData8(0x21); 

	SendCommand(0x8A);			
	SendData8(0x00); 

	SendCommand(0x8B);			
	SendData8(0x80); 

	SendCommand(0x8C);			
	SendData8(0x01); 

	SendCommand(0x8D);			
	SendData8(0x01); 

	SendCommand(0x8E);			
	SendData8(0xFF); 

	SendCommand(0x8F);			
	SendData8(0xFF); 
}

static inline void ConfigureBitsPerPixel()
{
	SendCommand(CMD_PIXEL_FORMAT_SET);			
	SendData8(0b0101); // 16 bits per pixel
}

static inline void ConfigureGamma()
{
	SendCommand(CMD_SET_GAMMA_1);   
	SendData8(0x45);
	SendData8(0x09);
	SendData8(0x08);
	SendData8(0x08);
	SendData8(0x26);
 	SendData8(0x2A);

 	SendCommand(CMD_SET_GAMMA_2);    
 	SendData8(0x43);
 	SendData8(0x70);
 	SendData8(0x72);
 	SendData8(0x36);
 	SendData8(0x37);  
 	SendData8(0x6F);

 	SendCommand(CMD_SET_GAMMA_3);   
 	SendData8(0x45);
 	SendData8(0x09);
 	SendData8(0x08);
 	SendData8(0x08);
 	SendData8(0x26);
 	SendData8(0x2A);

 	SendCommand(CMD_SET_GAMMA_4);   
 	SendData8(0x43);
 	SendData8(0x70);
 	SendData8(0x72);
 	SendData8(0x36);
 	SendData8(0x37); 
 	SendData8(0x6F);
}

void LcdScreen_Init(Timer* timer, ScanDirection scanDirection, LcdScreen* out)
{
    LcdScreen screen = 
    {
        .Base =
        {
			.SetViewport = SetViewport,
            .Clear = Clear,
            .DrawBuffer = DrawBuffer, 
			.SetBacklightPercentage = SetBacklightPercentage,
			.GetHeight = GetHeight,
			.GetWidth = GetWidth
        },
		.SetSleep = SetSleep,
		.SetIdleMode = SetIdleMode,
		.SetDisplayEnabled = SetDisplayEnabled,
		.Timer = timer
    };

    // LCD GPIO config
    SetUpGpioPin(LCD_RST_PIN, 1);
    SetUpGpioPin(LCD_DC_PIN, 1);
    SetUpGpioPin(LCD_CS_PIN, 1);
    SetUpGpioPin(LCD_BL_PIN, 1);

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
	screen.BacklightPwmSliceNumber = pwmSliceNum;

    // SPI Config
    spi_init(SPI_PORT, 40000 * 1000);
    gpio_set_function(LCD_CLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(LCD_MOSI_PIN, GPIO_FUNC_SPI);

    //Hardware reset
    Reset(&screen);

    memcpy(out, &screen, sizeof(LcdScreen));

    ConfigureSpi2DataControl();
    ConfigureUndocumentedCommands();
    ConfigureDisplayFunctionControl(false, true);
	ConfigureMemoryAccessCo(ntrol(0, 1, 1, 0, 1, 0); // Rotate 90 degrees
	ConfigureBitsPerPixel();

	SendCommand(0x90);			
	SendData8(0x08);
	SendData8(0x08);
	SendData8(0x08);
	SendData8(0x08); 

	SendCommand(0xBD);			
	SendData8(0x06);
	
	SendCommand(0xBC);			
	SendData8(0x00);	

	SendCommand(0xFF);			
	SendData8(0x60);
	SendData8(0x01);
	SendData8(0x04);

	SendCommand(0xC3);			
	SendData8(0x13);
	SendCommand(0xC4);			
	SendData8(0x13);

	SendCommand(0xC9);			
	SendData8(0x22);

	SendCommand(0xBE);			
	SendData8(0x11); 

	SendCommand(0xE1);			
	SendData8(0x10);
	SendData8(0x0E);

	SendCommand(0xDF);			
	SendData8(0x21);
	SendData8(0x0c);
	SendData8(0x02);
	
	ConfigureGamma();

	SendCommand(0xED);	
	SendData8(0x1B); 
	SendData8(0x0B); 

	SendCommand(0xAE);			
	SendData8(0x77);
	
	SendCommand(0xCD);			
	SendData8(0x63);		

	SendCommand(0x70);			
	SendData8(0x07);
	SendData8(0x07);
	SendData8(0x04);
	SendData8(0x0E); 
	SendData8(0x0F); 
	SendData8(0x09);
	SendData8(0x07);
	SendData8(0x08);
	SendData8(0x03);

	SendCommand(0xE8);			
	SendData8(0x34);

	SendCommand(0x62);			
	SendData8(0x18);
	SendData8(0x0D);
	SendData8(0x71);
	SendData8(0xED);
	SendData8(0x70); 
	SendData8(0x70);
	SendData8(0x18);
	SendData8(0x0F);
	SendData8(0x71);
	SendData8(0xEF);
	SendData8(0x70); 
	SendData8(0x70);

	SendCommand(0x63);			
	SendData8(0x18);
	SendData8(0x11);
	SendData8(0x71);
	SendData8(0xF1);
	SendData8(0x70); 
	SendData8(0x70);
	SendData8(0x18);
	SendData8(0x13);
	SendData8(0x71);
	SendData8(0xF3);
	SendData8(0x70); 
	SendData8(0x70);

	SendCommand(0x64);			
	SendData8(0x28);
	SendData8(0x29);
	SendData8(0xF1);
	SendData8(0x01);
	SendData8(0xF1);
	SendData8(0x00);
	SendData8(0x07);

	SendCommand(0x66);			
	SendData8(0x3C);
	SendData8(0x00);
	SendData8(0xCD);
	SendData8(0x67);
	SendData8(0x45);
	SendData8(0x45);
	SendData8(0x10);
	SendData8(0x00);
	SendData8(0x00);
	SendData8(0x00);

	SendCommand(0x67);			
	SendData8(0x00);
	SendData8(0x3C);
	SendData8(0x00);
	SendData8(0x00);
	SendData8(0x00);
	SendData8(0x01);
	SendData8(0x54);
	SendData8(0x10);
	SendData8(0x32);
	SendData8(0x98);

	SendCommand(0x74);			
	SendData8(0x10);	
	SendData8(0x85);	
	SendData8(0x80);
	SendData8(0x00); 
	SendData8(0x00); 
	SendData8(0x4E);
	SendData8(0x00);					
	
    SendCommand(0x98);			
	SendData8(0x3e);
	SendData8(0x07);

	SendCommand(0x35);	
	SendCommand(0x21);

	SendCommand(0x11);
	(*timer->WaitMilliseconds)(timer, 120);
	SendCommand(0x29);
	(*timer->WaitMilliseconds)(timer, 120);
}