#include "SdlHelper.h"
#include "MathHelper.h"

void SDL_RectToPoint(SDL_Rect* rect, SDL_Point* out)
{
    out->x = rect->x;
    out->y = rect->y;
}

int SDL_RectBottom(SDL_Rect* rect)
{
    return rect->y + rect->h;
}

int SDL_RectRight(SDL_Rect* rect)
{
    return rect->x + rect->w;
}

void SDL_RectMerge(SDL_Rect* l, SDL_Rect* r, SDL_Rect* out)
{
    // Break these out seperately in case on of the inputs are the same as the output.
    int x = Math_SignedMin(l->x, r->x);
    int y = Math_SignedMin(l->y, r->y);
    int w = Math_Max(SDL_RectRight(l), SDL_RectRight(r)) - out->x;
    int h = Math_Max(SDL_RectBottom(l), SDL_RectBottom(r)) - out->y;

    out->x = x;
    out->y = y;
    out->w = w;
    out->h = h;
}