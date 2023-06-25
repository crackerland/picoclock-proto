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

// #define WHITE          0xFFFF
// #define BLACK          0x0000
// #define BLUE           0x001F
// #define BRED           0XF81F
// #define GRED           0XFFE0
// #define ORANGE         0XFE20
// #define GBLUE          0X07FF
// #define RED            0xF800
// #define MAGENTA        0xF81F
// #define GREEN          0x07E0
// #define CYAN           0x7FFF
// #define YELLOW         0xFFE0
// #define BROWN          0XBC40
// #define BRRED          0XFC07
// #define GRAY           0X8430
// #define LGRAY          0X8551
// #define NBLACK         0x0821
// #define NWHITE         0xFFFE

extern _Colors Colors;

static void DrawPoint16(uint16_t color, unsigned int x, unsigned int y, Texture16* texture)
{
    texture->PixelData[y * texture->Base.TextureBuffer.Width + x] = color;
}

static void DrawPoint(Color color, unsigned int x, unsigned int y, Texture16* texture)
{
    DrawPoint16(Color_ToRgb565(color), x, y, texture);
}

void Paint_DrawCircle(
    uint16_t xCenter, 
    uint16_t yCenter, 
    uint16_t radius, 
    uint16_t color, 
    bool drawFilled,
    Texture16* texture)
{
    // Draw a circle from(0, R) as a starting point
    int16_t xCurrent = 0;
    int16_t yCurrent = radius;

    // Cumulative error,judge the next point of the logo
    int16_t esp = 3 - (radius << 1 );

    int16_t sCountY;
    if (drawFilled) 
    {
        while (xCurrent <= yCurrent) 
        {
            //Realistic circles
            for (sCountY = xCurrent; sCountY <= yCurrent; sCountY ++ ) 
            {
                DrawPoint16(color, xCenter + xCurrent, yCenter + sCountY, texture);//1
                DrawPoint16(color, xCenter - xCurrent, yCenter + sCountY, texture);//2
                DrawPoint16(color, xCenter - sCountY, yCenter + xCurrent, texture);//3
                DrawPoint16(color, xCenter - sCountY, yCenter - xCurrent, texture);//4
                DrawPoint16(color, xCenter - xCurrent, yCenter - sCountY, texture);//5
                DrawPoint16(color, xCenter + xCurrent, yCenter - sCountY, texture);//6
                DrawPoint16(color, xCenter + sCountY, yCenter - xCurrent, texture);//7
                DrawPoint16(color, xCenter + sCountY, yCenter + xCurrent, texture);
            }
            if (esp < 0 )
            {
                esp += 4 * xCurrent + 6;
            }
            else 
            {
                esp += 10 + 4 * (xCurrent - yCurrent);
                yCurrent--;
            }

            xCurrent++;
        }
    } 
    else 
    { //Draw a hollow circle
        while (xCurrent <= yCurrent ) 
        {
            DrawPoint16(color, xCenter + xCurrent, yCenter + yCurrent, texture);//1
            DrawPoint16(color, xCenter - xCurrent, yCenter + yCurrent, texture);//2
            DrawPoint16(color, xCenter - yCurrent, yCenter + xCurrent, texture);//3
            DrawPoint16(color, xCenter - yCurrent, yCenter - xCurrent, texture);//4
            DrawPoint16(color, xCenter - xCurrent, yCenter - yCurrent, texture);//5
            DrawPoint16(color, xCenter + xCurrent, yCenter - yCurrent, texture);//6
            DrawPoint16(color, xCenter + yCurrent, yCenter - xCurrent, texture);//7
            DrawPoint16(color, xCenter + yCurrent, yCenter + xCurrent, texture);//0

            if (esp < 0 )
            {
                esp += 4 * xCurrent + 6;
            }
            else 
            {
                esp += 10 + 4 * (xCurrent - yCurrent );
                yCurrent--;
            }

            xCurrent++;
        }
    }
}

static void Loop(AppResources* app)
{ 
    (*app->CanvasTexture->Clear)(app->CanvasTexture, Colors.Black);
    Paint_DrawCircle(
        app->CenterX, 
        app->CenterY, 
        (app->ScreenWidth / 2) - 1, 
        Color_ToRgb565(Colors.Yellow), 
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

// Test blocks of colors 

    // Paint_DrawCircle(
    //     app->CenterX, 
    //     app->CenterY, 
    //     (app->ScreenWidth / 2) - 1, 
    //     Color_ToRgb565(Colors.Yellow), 
    //     false, 
    //     (Texture16*)app->CanvasTexture);
}

void App_Init(
    Screen* screen, 
    Timer* timer, 
    DateTimeProvider* dateTimeProvider,
    PowerManager* powerManager,
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
            // .Loop = TestLoop,
            .Dispose = Dispose,
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
            .CenterY = screenHeight / 2
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
        &out->DateTextView);

    SystemFontTextView_Init(
        &out->TextureBuffer,
        Colors.Green, 
        Colors.Black, 
        &out->CanvasBufferRenderer.Base, 
        &out->TimeTextView);

    SystemFontTextView_Init(
        &out->TextureBuffer,
        Colors.Green, 
        Colors.Black, 
        &out->CanvasBufferRenderer.Base, 
        &out->BatteryTextView);

    SetUp(&out->Resources);
}