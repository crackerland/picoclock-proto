#include "PicoColorConverter.h"
#include "Color.h"

static uint16_t Convert565(ColorConverter* _, Color color)
{
    uint16_t converted = Color_ToRgb565(color);
    return converted >> 8 | (converted & 0xFF00) << 8;
}

static uint32_t Convert8888(ColorConverter* _, Color color)
{
    return 0;
}

void PicoColorConverter_Init(ColorConverter* out)
{
    out->Convert565 = Convert565;
    out->Convert8888 = Convert8888;
}