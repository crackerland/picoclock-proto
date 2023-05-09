#include "PicoClock.h"
#include <string.h>
#include <stdlib.h>
#include "DefaultDateTimeFormatter.h"
#include "ScreenTextureRenderer.h"
#include "BufferTextureRenderer.h"
#include "SystemFontTextView.h"
#include "Texture16.h"
#include "DateTime.h"
#include "MathHelper.h"
#include "ByteHelper.h"

#define WHITE          0xFFFF
#define BLACK          0x0000
#define BLUE           0x001F
#define BRED           0XF81F
#define GRED           0XFFE0
#define ORANGE         0XFE20
#define GBLUE          0X07FF
#define RED            0xF800
#define MAGENTA        0xF81F
#define GREEN          0x07E0
#define CYAN           0x7FFF
#define YELLOW         0xFFE0
#define BROWN          0XBC40
#define BRRED          0XFC07
#define GRAY           0X8430
#define LGRAY          0X8551
#define NBLACK         0x0821
#define NWHITE         0xFFFE

extern _Colors Colors;

static void DrawPoint16(uint16_t color, unsigned int x, unsigned int y, Texture16* texture)
{
    texture->PixelData[y * texture->TextureBuffer.Width + x] = color;
}

static void DrawPoint(Color color, unsigned int x, unsigned int y, Texture16* texture)
{
    DrawPoint16(Color_ToRgb565(color), x, y, texture);
}

void Paint_DrawCircle(
    uint16_t X_Center, 
    uint16_t Y_Center, 
    uint16_t Radius, 
    uint16_t Color, 
    bool Draw_Fill,
    Texture16* texture)
{
    Color = ByteHelper_Reverse16(Color);

    //Draw a circle from(0, R) as a starting point
    int16_t XCurrent, YCurrent;
    XCurrent = 0;
    YCurrent = Radius;

    //Cumulative error,judge the next point of the logo
    int16_t Esp = 3 - (Radius << 1 );

    int16_t sCountY;
    if (Draw_Fill) {
        while (XCurrent <= YCurrent ) { //Realistic circles
            for (sCountY = XCurrent; sCountY <= YCurrent; sCountY ++ ) {
                DrawPoint16(Color, X_Center + XCurrent, Y_Center + sCountY, texture);//1
                DrawPoint16(Color, X_Center - XCurrent, Y_Center + sCountY, texture);//2
                DrawPoint16(Color, X_Center - sCountY, Y_Center + XCurrent, texture);//3
                DrawPoint16(Color, X_Center - sCountY, Y_Center - XCurrent, texture);//4
                DrawPoint16(Color, X_Center - XCurrent, Y_Center - sCountY, texture);//5
                DrawPoint16(Color, X_Center + XCurrent, Y_Center - sCountY, texture);//6
                DrawPoint16(Color, X_Center + sCountY, Y_Center - XCurrent, texture);//7
                DrawPoint16(Color, X_Center + sCountY, Y_Center + XCurrent, texture);
            }
            if (Esp < 0 )
                Esp += 4 * XCurrent + 6;
            else {
                Esp += 10 + 4 * (XCurrent - YCurrent );
                YCurrent --;
            }
            XCurrent ++;
        }
    } else { //Draw a hollow circle
        while (XCurrent <= YCurrent ) {
            DrawPoint16(Color, X_Center + XCurrent, Y_Center + YCurrent, texture);//1
            DrawPoint16(Color, X_Center - XCurrent, Y_Center + YCurrent, texture);//2
            DrawPoint16(Color, X_Center - YCurrent, Y_Center + XCurrent, texture);//3
            DrawPoint16(Color, X_Center - YCurrent, Y_Center - XCurrent, texture);//4
            DrawPoint16(Color, X_Center - XCurrent, Y_Center - YCurrent, texture);//5
            DrawPoint16(Color, X_Center + XCurrent, Y_Center - YCurrent, texture);//6
            DrawPoint16(Color, X_Center + YCurrent, Y_Center - XCurrent, texture);//7
            DrawPoint16(Color, X_Center + YCurrent, Y_Center + XCurrent, texture);//0

            if (Esp < 0 )
                Esp += 4 * XCurrent + 6;
            else {
                Esp += 10 + 4 * (XCurrent - YCurrent );
                YCurrent --;
            }
            XCurrent ++;
        }
    }
}

static Size MeasureString(char* string, FontPainter* fontPainter)
{
    Size totalSize = { };
    int length = strlen(string);
    for (int i = 0; i < length; i++)
    {
        Size glyphSize = (*fontPainter->MeasureGlyph)(fontPainter, string[i]);
        totalSize.Width += glyphSize.Width;
        totalSize.Height = Math_Max(totalSize.Height, glyphSize.Height);
    }

    return totalSize;
}

static void Loop(AppResources* app)
{ 
    DateTime time;
    (*app->DateTimeProvider->GetDateTime)(app->DateTimeProvider, &time);

    (*app->DateTimeFormatter->GetDayAndMonth)(app->DateTimeFormatter, &time, app->DateTextView->Text);
    Size dateSize = (*app->DateTextView->Base.Measure)(&app->DateTextView->Base, app->ScreenWidth, app->ScreenHeight);
    (*app->DateTextView->Base.SetPositionX)(&app->DateTextView->Base, app->CenterX);
    (*app->DateTextView->Base.SetPositionY)(&app->DateTextView->Base, app->CenterY - dateSize.Height);
    (*app->DateTextView->Base.Draw)(&app->DateTextView->Base);

    (*app->DateTimeFormatter->GetHoursMinutesSeconds)(app->DateTimeFormatter, &time, app->TimeTextView->Text);
    Size timeSize = (*app->TimeTextView->Base.Measure)(&app->TimeTextView->Base, app->ScreenWidth, app->ScreenHeight);
    (*app->TimeTextView->Base.SetPositionX)(&app->TimeTextView->Base, app->CenterX - (timeSize.Width / 2));
    (*app->TimeTextView->Base.SetPositionY)(&app->TimeTextView->Base, app->CenterY + timeSize.Height);
    (*app->TimeTextView->Base.Draw)(&app->TimeTextView->Base);

    (*app->CanvasTexture->Draw)(app->CanvasTexture, 0, 0);

    (*app->Timer->WaitMilliseconds)(app->Timer, 1000);
}

static void Dispose(AppLifecycle* app)
{
    App* this = (App*)app;
    free(this->TextureBuffer.PixelData);
}

void App_Init(
    Screen* screen, 
    Timer* timer, 
    DateTimeProvider* dateTimeProvider,
    App* out)
{
    (*screen->Clear)(screen, BLACK);
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
        .Resources = 
        {
            .Screen = screen,
            .Timer = timer,
            .DateTimeProvider = dateTimeProvider,
            .CanvasTexture = &out->CanvasTexture.Base,
            .DateTextView = &out->DateTextView.Base,
            .TimeTextView = &out->TimeTextView.Base,
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

    (*out->CanvasTexture.Base.Clear)(&out->CanvasTexture.Base, 0x0);

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

    Paint_DrawCircle(out->Resources.CenterX, out->Resources.CenterY, (screenWidth / 2) - 1, YELLOW, false, &out->CanvasTexture);
}