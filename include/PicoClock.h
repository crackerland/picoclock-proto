#ifndef PicoClock_h
#define PicoClock_h

#include "PowerManager.h"
#include "Screen.h"
#include "Timer.h"
#include "DateTime.h"
#include "Texture16.h"
#include "DefaultDateTimeFormatter.h"
#include "SystemFontTextView.h"
#include "ScreenTextureRenderer.h"
#include "BufferTextureRenderer.h"
#include "PowerManager.h"

typedef struct AppResources
{
    Screen* Screen;
    Timer* Timer;
    DateTimeProvider* DateTimeProvider;
    DateTimeFormatter* DateTimeFormatter;
    Texture* CanvasTexture;
    TextView* DateTextView;
    TextView* TimeTextView;
    TextView* BatteryTextView;
    PowerManager* PowerManager;
    const unsigned int CenterX;
    const unsigned int CenterY;
    const unsigned int ScreenWidth;
    const unsigned int ScreenHeight;
}
AppResources;

typedef struct AppLifecycle
{
    void (*Loop)(struct AppResources*);
    void (*Dispose)(struct AppLifecycle*);
}
AppLifecycle;

typedef struct App
{
    AppLifecycle Lifecycle;
    AppResources Resources;
    DefaultDateTimeFormatter DateTimeFormatter;
    ScreenTextureRenderer ScreenRenderer;
    TextureBuffer TextureBuffer;
    BufferTextureRenderer CanvasBufferRenderer;
    SystemFontTextView DateTextView;
    SystemFontTextView TimeTextView;
    SystemFontTextView BatteryTextView;
    Texture16 CanvasTexture;
}
App;

extern void App_Init(    
    Screen* screen, 
    Timer* timer, 
    DateTimeProvider* dateTimeProvider,
    PowerManager* powerManager,
    App* out);

#endif
