#include "SdlScreen.h"
#include <string.h>

static void Clear(Screen* screen, uint16_t color)
{
    SdlScreen* this = (SdlScreen*)screen;
    SDL_SetRenderDrawColor(this->Renderer, 0, 0, 0xFF, 0xFF);
    SDL_RenderClear(this->Renderer);
    SDL_SetRenderDrawColor(this->Renderer, 0, 0, 0, 0);
}

static void SetViewport(Screen* screen, uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd)
{
    SdlScreen* this = (SdlScreen*)screen;
    this->Viewport.x = xStart;
    this->Viewport.y = yStart;
    this->Viewport.w = xEnd - xStart;
    this->Viewport.h = yEnd - yStart;
}

static void DrawBuffer(Screen* screen, uint8_t* buffer, size_t size)
{
    SdlScreen* this = (SdlScreen*)screen;

    SDL_UpdateTexture(
        this->Texture, 
        NULL, 
        buffer, 
        this->BytesPerPixel * this->Width);

    // Clear output.
    SDL_SetRenderTarget(this->Renderer, NULL);
    SDL_RenderClear(this->Renderer);

    SDL_RenderCopy(
        this->Renderer, 
        this->Texture, 
        NULL,
        &this->Viewport);

    SDL_RenderPresent(this->Renderer);
}

static void SetBacklightPercentage(Screen* screen, uint8_t percentage)
{
}

static unsigned int GetWidth(Screen* screen)
{
    SdlScreen* this = (SdlScreen*)screen;
    return this->Width;
}

static unsigned int GetHeight(Screen* screen)
{
    SdlScreen* this = (SdlScreen*)screen;
    return this->Height;
}

void SdlScreen_Init(SdlScreen* out, unsigned int width, unsigned int height)
{
    SDL_Window* window = SDL_CreateWindow(
        "Pico Clock", 
        SDL_WINDOWPOS_UNDEFINED, 
        SDL_WINDOWPOS_UNDEFINED, 
        width, 
        height, 
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    SDL_RenderSetLogicalSize(renderer, width, height);

    // uint32_t pixelFormat = SDL_GetWindowPixelFormat(window);
    SDL_Texture* texture = SDL_CreateTexture(
        renderer, 
        SDL_PIXELFORMAT_RGB565,
        SDL_TEXTUREACCESS_TARGET, 
        width, 
        height);

    uint32_t format = 0;
    int actualWidth = 0;
    int actualHeight = 0;
    SDL_QueryTexture(texture, &format, NULL, &actualWidth, &actualHeight);

// SDL_PixelFormatEnum

    SdlScreen screen = 
    {
        .Base = 
        {
			.SetViewport = SetViewport,
            .Clear = Clear,
            .DrawBuffer = DrawBuffer, 
			.SetBacklightPercentage = SetBacklightPercentage,
			.GetHeight = GetHeight,
			.GetWidth = GetWidth
        },
        .Window = window,
        .Renderer = renderer,
        .Texture = texture,
        .Viewport = (SDL_Rect){ .w = width, .h = height },
        .BytesPerPixel = SDL_BYTESPERPIXEL(format),
        .Width = actualWidth,
        .Height = actualHeight
    };

    memcpy(out, &screen, sizeof(screen));
}