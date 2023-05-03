#ifndef SdlScreen_h
#define SdlScreen_h

#include "Screen.h"
#include "SDL.h"

typedef struct SdlScreen
{
    Screen Base;
    SDL_Window* Window;
    SDL_Renderer* Renderer;
    SDL_Texture* Texture;
    SDL_Rect Viewport;
    unsigned int BytesPerPixel;
    unsigned int Width;
    unsigned int Height;
}
SdlScreen;

extern void SdlScreen_Init(SdlScreen* out, unsigned int width, unsigned int height);

#endif