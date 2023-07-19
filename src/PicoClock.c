#include "PicoClock.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "DefaultDateTimeFormatter.h"
#include "ScreenTextureRenderer.h"
#include "BufferTextureRenderer.h"
#include "SystemFontTextView.h"
#include "Texture16.h"
#include "DateTime.h"
#include "MathHelper.h"
#include "ByteHelper.h"

extern _Colors Colors;

static inline void HandleInput(PendingInput* pending, AppResources* app)
{
    if (pending->Minus)
    {
        printf("Polled input: MINUS\n");
    }

    if (pending->Plus)
    {
        printf("Polled input: PLUS\n");
    }

    if (pending->Select)
    {
        printf("Polled input: SELECT\n");
    }

    if (pending->Sleep)
    {
        printf("Polled input: SLEEP\n");
        (*app->PowerManager->Sleep)(app->PowerManager);
    }

    if (pending->Reset)
    {
        printf("Polled input: RESET\n");
        (*app->PowerManager->Reset)(app->PowerManager);
    }

    PendingInput cleared = { };
    memcpy(pending, &cleared, sizeof(PendingInput));
}

static void Loop(AppResources* app)
{ 
    HandleInput(app->PendingInput, app);

    (*app->CanvasTexture->Clear)(app->CanvasTexture, Colors.Black);
    (*app->Painter.DrawCircle)(
        &app->Painter,
        app->CenterX, 
        app->CenterY, 
        (app->ScreenWidth / 2) - 1, 
        Colors.Yellow,
        false, 
        (Texture16*)app->CanvasTexture);

    DateTime time;
    (*app->DateTimeProvider->GetDateTime)(app->DateTimeProvider, &time);

    (*app->DateTimeFormatter->GetDayAndMonth)(app->DateTimeFormatter, &time, app->DateTextView->Text);
    Size textViewSize = (*app->DateTextView->Base.Measure)(&app->DateTextView->Base, app->ScreenWidth, app->ScreenHeight);
    (*app->DateTextView->Base.SetPositionX)(&app->DateTextView->Base, app->CenterX - (textViewSize.Width / 2));
    (*app->DateTextView->Base.SetPositionY)(&app->DateTextView->Base, app->CenterY - textViewSize.Height);
    (*app->DateTextView->Base.Draw)(&app->DateTextView->Base);

    (*app->DateTimeFormatter->GetHoursMinutesSeconds)(app->DateTimeFormatter, &time, app->TimeTextView->Text);
    textViewSize = (*app->TimeTextView->Base.Measure)(&app->TimeTextView->Base, app->ScreenWidth, app->ScreenHeight);
    (*app->TimeTextView->Base.SetPositionX)(&app->TimeTextView->Base, app->CenterX - (textViewSize.Width / 2));
    (*app->TimeTextView->Base.SetPositionY)(&app->TimeTextView->Base, app->CenterY + textViewSize.Height);
    (*app->TimeTextView->Base.Draw)(&app->TimeTextView->Base);

    Battery* battery = (*app->PowerManager->GetBattery)(app->PowerManager);
    float voltage = (*battery->Read)(battery);
    sprintf(app->BatteryTextView->Text, "%.1f v.", voltage);
    textViewSize = (*app->BatteryTextView->Base.Measure)(&app->BatteryTextView->Base, app->ScreenWidth, app->ScreenHeight);
    (*app->BatteryTextView->Base.SetPositionX)(&app->BatteryTextView->Base, app->CenterX - (textViewSize.Width / 2));
    (*app->BatteryTextView->Base.SetPositionY)(&app->BatteryTextView->Base, textViewSize.Height * 2);
    (*app->BatteryTextView->Base.Draw)(&app->BatteryTextView->Base);

    (*app->CanvasTexture->Draw)(app->CanvasTexture, 0, 0);
}

static void Dispose(AppLifecycle* app)
{
    App* this = (App*)app;
    free(this->TextureBuffer.PixelData);
}

static void TestLoop(AppResources* app)
{
    (*app->CanvasTexture->Draw)(app->CanvasTexture, 0, 0);
}

static void SetUp(AppResources* app)
{
    // Red - good
    // Green - bad (shows blue)
    // Blue - bad (shows green)
    (*app->CanvasTexture->Clear)(app->CanvasTexture, Colors.Cyan);
}

static void Plus(UserInput* input)
{
    AppUserInput* this = (AppUserInput*)input;
    this->Pending.Plus = true;
}

static void Minus(UserInput* input)
{
    AppUserInput* this = (AppUserInput*)input;
    this->Pending.Minus = true;
}

static void Select(UserInput* input)
{
    AppUserInput* this = (AppUserInput*)input;
    this->Pending.Select = true;
}

static void Sleep(UserInput* input)
{
    AppUserInput* this = (AppUserInput*)input;
    this->Pending.Sleep = true;
}

void App_Init(
    Screen* screen, 
    Timer* timer, 
    DateTimeProvider* dateTimeProvider,
    PowerManager* powerManager,
    ColorConverter* colorConverter,
    App* out)
{
    (*screen->Clear)(screen, Colors.Black);
    (*screen->SetBacklightPercentage)(screen, 50);
    unsigned int screenWidth = (*screen->GetWidth)(screen);
    unsigned int screenHeight = (*screen->GetHeight)(screen);

    App app = 
    {
        .Lifecycle =
        {
            .Loop = Loop,
            .Dispose = Dispose,
        },
        .Input = 
        {
            .Base = 
            {
                .Minus = Minus,
                .Plus = Plus,
                .Select = Select,
                .Sleep = Sleep
            }
        },
        .Resources = 
        {
            .Screen = screen,
            .Timer = timer,
            .DateTimeProvider = dateTimeProvider,
            .PowerManager = powerManager,
            .CanvasTexture = &out->CanvasTexture.Base.Base,
            .DateTextView = &out->DateTextView.Base,
            .TimeTextView = &out->TimeTextView.Base,
            .BatteryTextView = &out->BatteryTextView.Base,
            .DateTimeFormatter = &out->DateTimeFormatter.Base,
            .ScreenWidth = screenWidth,
            .ScreenHeight = screenHeight,
            .CenterX = screenWidth / 2,
            .CenterY = screenHeight / 2,
            .PendingInput = &out->Input.Pending
        },
        .TextureBuffer = 
        {
            .PixelData = (uint16_t*)malloc(sizeof(uint16_t) * screenWidth * screenHeight),
            .Width = screenWidth,
            .Height = screenHeight,
            .Stride = screenWidth * sizeof(uint16_t)
        }
    };

    memcpy(out, &app, sizeof(app));

    Painter_Init(colorConverter, &out->Resources.Painter);

    DefaultDateTimeFormatter_Init(&out->DateTimeFormatter);
    DateTimeFormatter* dateTimeFormatter = &out->DateTimeFormatter.Base;

    ScreenTextureRenderer_Init(screen, &out->ScreenRenderer);

    Texture16_Init(
        out->TextureBuffer.PixelData,
        screenWidth, 
        screenHeight, 
        &out->ScreenRenderer.Base, 
        &out->CanvasTexture);

    BufferTextureRenderer_Init(
        &out->TextureBuffer,
        &out->CanvasBufferRenderer);

    SystemFontTextView_Init(
        &out->TextureBuffer,
        Colors.Green, 
        Colors.Black, 
        &out->CanvasBufferRenderer.Base,
        colorConverter,
        &out->DateTextView);

    SystemFontTextView_Init(
        &out->TextureBuffer,
        Colors.Green, 
        Colors.Black, 
        &out->CanvasBufferRenderer.Base, 
        colorConverter,
        &out->TimeTextView);

    SystemFontTextView_Init(
        &out->TextureBuffer,
        Colors.Green, 
        Colors.Black, 
        &out->CanvasBufferRenderer.Base, 
        colorConverter,
        &out->BatteryTextView);

    SetUp(&out->Resources);
}