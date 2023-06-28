#ifndef Painter_h
#define Painter_h

#include "Texture16.h"
#include "ColorConverter.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct Painter
{
    void (*DrawPoint)(struct Painter*, Color color, unsigned int x, unsigned int y, Texture16* texture);
    void (*DrawCircle)(
        struct Painter*,
        uint16_t xCenter, 
        uint16_t yCenter, 
        uint16_t radius, 
        Color color, 
        bool drawFilled,
        Texture16* texture);

    ColorConverter* ColorConverter;
}
Painter;

extern void Painter_Init(ColorConverter* colorConverter, Painter* out);

#endif