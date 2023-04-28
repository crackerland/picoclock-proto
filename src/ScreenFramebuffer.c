#include "ScreenFramebuffer.h"
#include <string.h>

static Task* WriteBuffer(Framebuffer* framebuffer, void* input, size_t size)
{
    ScreenFramebuffer* this = (ScreenFramebuffer*)framebuffer;
    (*this->Screen->DrawBuffer)(this->Screen, input, size);
}

static unsigned int GetWidth(Framebuffer* framebuffer)
{
    ScreenFramebuffer* this = (ScreenFramebuffer*)framebuffer;
    return (*this->Screen->GetWidth)(this->Screen);
}

static unsigned int GetHeight(Framebuffer* framebuffer)
{
    ScreenFramebuffer* this = (ScreenFramebuffer*)framebuffer;
    return (*this->Screen->GetHeight)(this->Screen);
}

static unsigned int GetVirtualWidth(Framebuffer* framebuffer)
{
    return GetWidth(framebuffer);
}

static unsigned int GetVirtualHeight(Framebuffer* framebuffer)
{
    return GetHeight(framebuffer);
}

static unsigned int GetBitsPerPixel(Framebuffer* framebuffer)
{
    return 16;
}

static unsigned int GetPitch(Framebuffer* framebuffer)
{
    return GetWidth(framebuffer) * (GetBitsPerPixel(framebuffer) / 8);
}

static Color GetPixel(Framebuffer* framebuffer, Point point)
{
    return (Color) { };
}

static void DrawPixel(Framebuffer* framebuffer, Color color, Point point)
{
    Screen* screen = ((ScreenFramebuffer*)framebuffer)->Screen;
    (*screen->SetViewport)(screen, point.X, point.Y, point.X, point.Y);

    uint16_t rgb565color = Color_ToRgb565(color);
    (*screen->DrawBuffer)(screen, (uint8_t*)&rgb565color, sizeof(uint16_t));
}

static void Clear(Framebuffer* framebuffer, Color color)
{
    Screen* screen = ((ScreenFramebuffer*)framebuffer)->Screen;
    (*screen->Clear)(screen, Color_ToRgb565(color));
}

static void SetViewportOffset(Framebuffer* framebuffer, unsigned int offset)
{
}

void ScreenFramebuffer_Init(Screen* screen, ScreenFramebuffer* out)
{
    ScreenFramebuffer framebuffer =
    {
        .Base = 
        {
            .GetWidth = GetWidth,
            .GetHeight = GetHeight,
            .GetVirtualWidth = GetVirtualWidth,
            .GetVirtualHeight = GetVirtualHeight,
            .GetBitsPerPixel = GetBitsPerPixel,
            .GetPitch = GetPitch,
            .WriteBuffer = WriteBuffer,
            .DrawPixel = DrawPixel,
            .GetPixel = GetPixel,
            .Clear = Clear,
            .SetViewportOffset = SetViewportOffset
        },
        .Screen = screen
    };

    memcpy(out, &framebuffer, sizeof(ScreenFramebuffer));
}