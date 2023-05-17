#ifndef PicoConsole_h
#define PicoConsole_h

#include "Console.h"
#include "hardware/uart.h"

typedef struct PicoConsole
{
    Console Base;
    uart_inst_t* Uart;
}
PicoConsole;

extern void PicoConsole_Init(uart_inst_t* uart, PicoConsole* out);

#endif