#include "FramebufferScreen.h"
#include <string.h>

static void Clear(Screen* screen, Color color)
{
    FramebufferScreen* this = (FramebufferScreen*)screen;
    (*this->Framebuffer->Clear)(this->Framebuffer, color);
}

static void SetViewport(Screen* screen, uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd)
{
    FramebufferScreen* this = (FramebufferScreen*)screen;
}

static void DrawBuffer(Screen* screen, uint8_t* buffer, size_t size)
{
    FramebufferScreen* this = (FramebufferScreen*)screen;
    (*this->Framebuffer->WriteBuffer)(this->Framebuffer, buffer, size);
}

static void SetBacklightPercentage(Screen* screen, uint8_t percentage)
{
}

static unsigned int GetWidth(Screen* screen)
{
    FramebufferScreen* this = (FramebufferScreen*)screen;
    return (*this->Framebuffer->GetWidth)(this->Framebuffer);
}

static unsigned int GetHeight(Screen* screen)
{
    FramebufferScreen* this = (FramebufferScreen*)screen;
    return (*this->Framebuffer->GetHeight)(this->Framebuffer);
}

void FramebufferScreen_Init(Framebuffer* framebuffer, FramebufferScreen* out)
{
    FramebufferScreen screen = 
    {
        .Base = 
        {
            .Clear = Clear,
            .DrawBuffer = DrawBuffer,
            .GetHeight = GetHeight,
            .GetWidth = GetWidth,
            .SetBacklightPercentage = SetBacklightPercentage,
            .SetViewport = SetViewport   
        },
        .Framebuffer = framebuffer
    };

    memcpy(out, &screen, sizeof(FramebufferScreen));
}