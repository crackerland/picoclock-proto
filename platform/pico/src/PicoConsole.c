#include "PicoConsole.h"
#include <string.h>
#include "pico/stdlib.h"

static char ReadChar(Console* console)
{
    PicoConsole* this = (PicoConsole*)console;
    uart_getc(this->Uart);
}

static void WriteChar(Console* console, char value)
{
    PicoConsole* this = (PicoConsole*)console;
    uart_putc(this->Uart, value);
}

static bool HasUnreadData(Console* console)
{
    PicoConsole* this = (PicoConsole*)console;
    return uart_is_readable(this->Uart);
}

static void WriteString(Console* console, const char* string)
{
    PicoConsole* this = (PicoConsole*)console;
    uart_puts(this->Uart, string);
}

void PicoConsole_Init(uart_inst_t* uart, PicoConsole* out)
{
    PicoConsole console = 
    {
        .Base = 
        {
            .ReadChar = ReadChar,
            .WriteChar = WriteChar,
            .HasUnreadData = HasUnreadData
            .WriteString = WriteString
        },
        .Uart = uart
    };

    memcpy(out, &console, sizeof(console));
}