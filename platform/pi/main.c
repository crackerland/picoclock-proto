#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "PicoClock.h"
#include "FramebufferScreen.h"
#include "SystemTimerDateTimeProvider.h"
#include "BcmBoard.h"
#include "RpiDma.h"
#include "RpiBase.h"
#include "cstubs.h"
#include "FramebufferFontPainter.h"
#include "FontPainterEx.h"
#include "Timer.h"
#include "Filesystem.h"
#include "StreamBitmap.h"
#include "StreamPng.h"
#include "interrupts.h"
#include "Size.h"

extern _Colors Colors;

static void PrintToConsole(int file, char *string, int length, void* userData)
{
    Uart* uart = (Uart*)userData;
    for (int todo = 0; todo < length; todo++)
    {
        (*uart->PutChar)(uart, *string++);
    }
}

static void ReadFromConsole(int file, char *string, int length, void* userData)
{
    Uart* uart = (Uart*)userData;
    *string = (uart->GetChar)(uart);
}

static void PrintToScreen(int file, char *string, int length, void* userData)
{
    FontPainter* painter = (FontPainter*)userData;
    char buf[1024] = { };
    memcpy(buf, string, length);
    (*painter->DrawString)(painter, buf);
}

void End()
{
    /* The base addresses of the peripheral registers (ARM Physical Address) */
    #define GPIO_BASE   ( PERIPHERAL_BASE + 0x200000UL )
    GpioPeripheral* peripheral = (GpioPeripheral*)GPIO_BASE;

    static const int bitsPerPin = 3;
    static const int functionSelectMask = 0b111;
    static const int pinsPerRegister = 10;
    unsigned int pin = 47;
    peripheral->GPFSEL4 = GpioPinFunction_Output << ((pin % pinsPerRegister) * bitsPerPin);
    peripheral->GPCLR1 = 1 << (pin - 32);
    // peripheral->GPSET1 = 1 << (pin - 32);
    while (1) {}
}

static void InitializeBoard(BcmBoardImpl* boardImpl)
{
    BcmBoard_Init(boardImpl);
    BcmBoard* board = &boardImpl->Base;

    (*boardImpl->Interrupts.Base.EnableInterrupts)(&boardImpl->Interrupts.Base);

    // Enable printf.
    Uart* uart = (*board->GetUart)(board);
    (*uart->Enable)(uart, 115200, UartBitMode_8Bit);

    ReadWriteDelegate consoleWriteDelegate = { .Handle = PrintToConsole, .UserData = &boardImpl->Uart.Base };
    CStubs_AddWriteHandler(&consoleWriteDelegate);

    ReadWriteDelegate consoleReadDelegate = { .Handle = ReadFromConsole, .UserData = &boardImpl->Uart.Base };
    CStubs_AddReadHandler(&consoleReadDelegate);

    // Set full CPU speed.
    ArmClock* armClock = (*board->GetArmClock)(board);
    (*armClock->SetClockRate)(armClock, (*armClock->GetMaxClockRate)(armClock));
}

static const float Battery_Read(Battery* battery)
{
    return 4.2;
}

Battery battery = { .Read = Battery_Read };

static void PowerManager_Sleep(PowerManager* powerManager)
{
}

static void PowerManager_WakeUp(PowerManager* powerManager)
{
}

static Battery* PowerManager_GetBattery(PowerManager* powerManager)
{
    return &battery;
}

static uint16_t Convert565(ColorConverter* _, Color color)
{
    return Color_ToRgb565(color);
}

static uint32_t Convert8888(ColorConverter* _, Color color)
{
    return 0;
}

void kernel_main(unsigned int r0, unsigned int r1, unsigned int atags)
{
    BcmBoardImpl boardImpl;
    InitializeBoard(&boardImpl);
    BcmBoard* board = &boardImpl.Base;

    Display display =
    {
        .Size = 
        {
            .Width = 320,
            .Height = 240,
        },
        .PixelFormat = FramebufferPixelFormat_RGB16
    };

    Framebuffer* framebuffer = (*board->CreateFramebuffer)(
        board, 
        &display, 
        display.Size);

    FramebufferFontPainter fontPainter;
    FramebufferFontPainter_Init(
        framebuffer, 
        Colors.Green,
        Colors.Black,
        &fontPainter);

    Timer* timer = (*board->GetSystemTimer)(board);

    FramebufferScreen screen;
    FramebufferScreen_Init(framebuffer, &screen);

    SystemTimerDateTimeProvider dateTimeProvider;
    SystemTimerDateTimeProvider_Init(timer, &dateTimeProvider);

    PowerManager powerManager =
    {
        .Sleep = PowerManager_Sleep,
        .WakeUp = PowerManager_WakeUp,
        .GetBattery = PowerManager_GetBattery
    };

    ColorConverter colorConverter = 
    {
        .Convert565 = Convert565,
        .Convert8888 = Convert8888,
    };

    App app;
    App_Init(&screen.Base, timer, &dateTimeProvider.Base, &powerManager, &colorConverter, &app);
    while(1)
    {
        (*app.Lifecycle.Loop)(&app.Resources);
        (*timer->WaitMilliseconds)(timer, 1000);
    }
}