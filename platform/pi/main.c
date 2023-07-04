#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

static File* selectFile(BcmBoard* bcmBoard)
{
    SdCard* sd = (*bcmBoard->GetSdCard)(bcmBoard);
    Directory* root = RPI_GetRootDirectory(sd);
    FileList* files = (*root->ListFiles)(root);
    unsigned int filecount = (*files->GetCount)(files);
    for (unsigned int i = 0; i < filecount; i++)
    {
        File* file = (*files->Get)(files, i);
        printf("[%u] File: %s\n", i, (*file->GetName)(file));
    }

    char indexBuffer[5] = { 0 };
    int charCount = 0;
    while (1)
    {
        char c = getchar();
        if (c == '\n')
        {
            if (!charCount)
            {
                printf("Invalid selection\n");
            }

            printf("\n");
            break;
        }

        int digit = c - '0'; // ASCII -> int
        if (digit < 0)
        {
            printf("Not a valid index\n");
            continue;
        }

        indexBuffer[charCount++] = c;
    }

    indexBuffer[charCount] = '\0';

    int index = atoi(indexBuffer);

    return (*files->Get)(files, index);
}

static void DrawLine(FontPainter* fontPainter, unsigned int columnCount, unsigned int y)
{
    (*fontPainter->SetCursorPosition)(fontPainter, (Point) { .X = 0, .Y = y });
    for (unsigned int x = 0; x < columnCount; x++)
    {
        (*fontPainter->DrawChar)(fontPainter, (unsigned char)(rand() % 2 ? '0' : '1'));
    }
}

static void Matrix(FontPainter* fontPainter, Framebuffer* framebuffer, Timer* timer)
{
    unsigned int lineSpacing = (*fontPainter->GetLineSpacing)(fontPainter);
    unsigned int lineHeight = 8 + lineSpacing;
    unsigned int vHeight = (*framebuffer->GetVirtualHeight)(framebuffer);
    unsigned int lineCount = vHeight / lineHeight;
    unsigned int frameHeight = vHeight - (*framebuffer->GetHeight)(framebuffer);
    unsigned int width = (*framebuffer->GetWidth)(framebuffer);
    unsigned int columnWidth = (8 + (*fontPainter->GetLetterSpacing)(fontPainter));
    unsigned int columnCount = width / columnWidth;
    for (unsigned int y = 0; y < lineCount; y++)
    {
        (*fontPainter->SetCursorPosition)(fontPainter, (Point){ .X = 0, .Y = y * lineHeight });

        unsigned char c[11];
        sprintf(c, "%u", y);
        (*fontPainter->DrawString)(fontPainter, c);
    }

    (*framebuffer->SetViewportOffset)(framebuffer, 0);

    for (unsigned int y = 0; y < lineCount; y++)
    {
        DrawLine(fontPainter, columnCount, y * lineHeight);
    }
    
    unsigned int drawY = 0;
    while (1)
    {
        for (unsigned int i = 0; i < frameHeight; i++)
        {
            (*fontPainter->SetCursorPosition)(
                fontPainter, 
                (Point)
                { 
                    .X = (rand() % width) * columnWidth, 
                    .Y = (rand() % vHeight) * lineHeight 
                });

            (*fontPainter->DrawChar)(fontPainter, (unsigned char)(rand() % 2 ? '0' : '1'));
                (*framebuffer->SetViewportOffset)(framebuffer, i);
                (*timer->WaitMilliseconds)(timer, 50);
        }
    }
}

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

static void TestDma(DmaController* dma)
{
    // printf("DMA start\n");

    // void* dmaBuf = malloc(sizeof(Color) * pixels);
    // DmaTransaction* tx = (*dma->InitTransaction)(dma, buffer, dmaBuf, sizeof(Color) * pixels);
    // DmaTask* task = (*tx->Start)(tx);
    // (*task->Base.Wait)(&task->Base, timer);

    // void* manualBuf = malloc(sizeof(Color) * pixels);
    // memcpy(manualBuf, buffer, sizeof(Color) * pixels);

    // for (int x = 0; x < 128; x++)
    // {
    //     Color* c = &buffer[x];
    //     printf(
    //         "%d: %08lX %08lX %08lX\n", 
    //         x,
    //         *((uint32_t*)c), 
    //         (((uint32_t*)dmaBuf)[x]), 
    //         ((uint32_t*)manualBuf)[x]);
    // }

    // printf("DMA complete\n");

    // free(buffer);
    // free(dmaBuf);
    // free(manualBuf);
}

static void DrawImage(File* file, Framebuffer* framebuffer, Timer* timer)
{
    char* name = (*file->GetName)(file);
    char extension[4];
    extension[3] = '\0';

    memcpy(extension, name + strlen(name) - 3, 3);

    Stream *stream = (*file->OpenRead)(file);
    Image* image;
    if (!strcmp(extension, "BMP"))
    {
        printf("BMP\n");
        StreamBitmap streamBmp; 
        StreamBitmap_Init(stream, &streamBmp);
        image = &streamBmp.Base.Base; 
    }
    else if (!strcmp(extension, "PNG"))
    {
        printf("PNG\n");
        StreamPng streamPng; 
        StreamPng_Init(stream, &streamPng);
        image = &streamPng.Base.Base;
    }

    unsigned int width = (*image->GetWidth)(image);
    unsigned int height = (*image->GetHeight)(image);
    printf("Width %u Height %u\n", width, height);
    unsigned int pixels = width * height;
    Color* buffer = malloc(sizeof(Color) * pixels);
    (*image->Read)(image, buffer);

    Task* task = (*framebuffer->WriteBuffer)(framebuffer, buffer, pixels);
    (*task->Wait)(task, timer);

    free(buffer);
}

static void DrawRect(
    Framebuffer* framebuffer, 
    Color color,
    unsigned int x, 
    unsigned int y, 
    unsigned int width,
    unsigned int height)
{
    unsigned int endY = y + height;
    unsigned int endX = x + width;
    unsigned int startX = x;
    for (; y < endY; y++)
    {
        x = startX;
        for (; x < endX; x++)
        {
            (*framebuffer->DrawPixel)(
                framebuffer, 
                color,
                (Point) { .X = x, .Y = y });
        }
    }
}

void kernel_main(unsigned int r0, unsigned int r1, unsigned int atags)
{
    BcmBoardImpl boardImpl;
    BcmBoard_Init(&boardImpl);
    BcmBoard* board = &boardImpl.Base;

    (*boardImpl.Interrupts.Base.EnableInterrupts)(&boardImpl.Interrupts.Base);

    // Enable printf.
    Uart* uart = (*board->GetUart)(board);
    (*uart->Enable)(uart, 115200, UartBitMode_8Bit);

    ReadWriteDelegate consoleWriteDelegate = { .Handle = PrintToConsole, .UserData = &boardImpl.Uart.Base };
    CStubs_AddWriteHandler(&consoleWriteDelegate);

    ReadWriteDelegate consoleReadDelegate = { .Handle = ReadFromConsole, .UserData = &boardImpl.Uart.Base };
    CStubs_AddReadHandler(&consoleReadDelegate);

    // Set full CPU speed.
    ArmClock* armClock = (*board->GetArmClock)(board);
    (*armClock->SetClockRate)(armClock, (*armClock->GetMaxClockRate)(armClock));

    Display display =
    {
        .Size = 
        {
            // .Width = 320,
            // .Height = 240,
            // .Width = 640,
            // .Height = 480,
            .Width = 512,
            .Height = 512
            // .Width = 1920,
            // .Height = 1080,
            // .Width = 3840,
            // .Height = 2160
        },
        .PixelFormat = FramebufferPixelFormat_ARGB32
    };

    Framebuffer* framebuffer = (*board->CreateFramebuffer)(
        board, 
        &display, 
        display.Size);
        // ((Size) { .Width = display.Size.Width, .Height = display.Size.Height * 2}));

    FramebufferFontPainter fontPainter;
    FramebufferFontPainter_Init(
        framebuffer, 
        Colors.Green,
        Colors.Black,
        &fontPainter);

    // Enable printf to screen.
    // ReadWriteDelegate screenPrintDelegate = { .Handle = PrintToScreen, .UserData = &fontPainter.Base };
    // CStubs_AddWriteHandler(&screenPrintDelegate);

    // uint8_t source[256];
    // memset(source, 0xFF, 256);
    // uint8_t dest[256] = {};

    // DmaTransaction* transaction = (*dma.Base.InitTransaction)(&dma.Base, source, dest, 128);
    // DmaTask* task = (*transaction->Start)(transaction);
    // (*task->Wait)(task, (*board->GetSystemTimer)(board));

    while(1)
    {
        Timer* timer = (*board->GetSystemTimer)(board);
        uint32_t time = (*timer->GetCurrentCounter)(timer);
        File *file = selectFile(board);

        printf("TIME %lu\n", ((*timer->GetCurrentCounter)(timer) - time) / 1000);
        unsigned long size = (*file->GetSize)(file);
        printf("******* %s %lu *******\n", (*file->GetName)(file), size);

        DrawImage(file, framebuffer, (*board->GetSystemTimer)(board));
        printf("Draw finished\n");
    }
}