#ifndef SdlHelper_h
#define SdlHelper_h

#include "SDL.h"

extern void SDL_RectToPoint(SDL_Rect* rect, SDL_Point* out);
extern int SDL_RectBottom(SDL_Rect* rect);
extern int SDL_RectRight(SDL_Rect* rect);
extern void SDL_RectMerge(SDL_Rect* l, SDL_Rect* r, SDL_Rect* out);

#endif