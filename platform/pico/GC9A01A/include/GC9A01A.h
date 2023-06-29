#ifndef GC9A01A_h
#define GC9A01A_h

#include "Screen.h"
#include "Timer.h"
#include <stdbool.h>

#define LCD_1IN28_HEIGHT 240
#define LCD_1IN28_WIDTH 240

typedef struct LcdAttributes
{
	uint16_t Width;
	uint16_t Height;
	uint8_t ScanDir;
}
LcdAttributes;

typedef enum ScanDirection
{
    ScanDirection_Horizontal,
    ScanDirection_Vertical
}
ScanDirection;

typedef struct LcdScreen
{
    Screen Base;
    void (*SetSleep)(struct LcdScreen*, bool sleep);

    // 6.2.21. Idle Mode:
    // "In the idle on mode, color expression is reduced. The primary and the secondary colors using
    // MSB of each R, G and B in the Frame Memory, 8 color depth data is displayed."
    void (*SetIdleMode)(struct LcdScreen*, bool sleep);

    // 6.2.9. Display OFF/6.2.10. Display ON:
    // "This command is used to enter into DISPLAY OFF mode. In this mode, the output from
    // Frame Memory is disabled and blank page inserted.
    // This command makes no change of contents of frame memory.
    // This command does not change any other status.
    // There will be no abnormal visible effect on the display."
    void (*SetDisplayEnabled)(struct LcdScreen*, bool enable);
	unsigned int BacklightPwmSliceNumber;
    LcdAttributes Attributes;
    uint8_t CurrentBacklightPercentage;
    Timer* Timer;
}
LcdScreen;

extern void LcdScreen_Init(Timer* timer, ScanDirection scanDirection, LcdScreen* out);

#endif