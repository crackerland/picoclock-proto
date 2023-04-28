#ifndef LcdScreen_h
#define LcdScreen_h

#include "Screen.h"

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
	unsigned int BacklightPwnSliceNumber;
    LcdAttributes Attributes;
}
LcdScreen;

extern void LcdScreen_Init(LcdScreen* out, ScanDirection scanDirection);

#endif