#ifndef FramebufferScreen_h
#define FramebufferScreen_h

#include "Screen.h"
#include "Framebuffer.h"

typedef struct FramebufferScreen 
{
    Screen Base;
    Framebuffer* Framebuffer;
}
FramebufferScreen;

extern void FramebufferScreen_Init(Framebuffer* framebuffer, FramebufferScreen* out);

#endif