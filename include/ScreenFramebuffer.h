#ifndef ScreenFramebuffer_h
#define ScreenFramebuffer_h

#include "Framebuffer.h"
#include "Screen.h"

typedef struct ScreenFramebuffer
{
    Framebuffer Base;
    Screen* Screen;
}
ScreenFramebuffer;

extern void ScreenFramebuffer_Init(Screen* screen, ScreenFramebuffer* out);

#endif