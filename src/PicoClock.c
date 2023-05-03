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
    texture->PixelData[y * texture->Width + x] = color;
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

void PicoClock_Start(
    Screen* screen, 
    Timer* timer, 
    DateTimeProvider* dateTimeProvider)
{
    DefaultDateTimeFormatter formatter;
    DefaultDateTimeFormatter_Init(&formatter);
    DateTimeFormatter* dateTimeFormatter = &formatter.Base;

    (*screen->Clear)(screen, BLACK);
    (*screen->SetBacklightPercentage)(screen, 50);
    unsigned int screenWidth = (*screen->GetWidth)(screen);
    unsigned int screenHeight = (*screen->GetHeight)(screen);

    ScreenTextureRenderer textureRenderer;
    ScreenTextureRenderer_Init(screen, &textureRenderer);

    uint16_t canvasTextureData[screenWidth * screenHeight];
    Texture16 canvasTexture;
    Texture16_Init(
        canvasTextureData, 
        screenWidth, 
        screenHeight, 
        &textureRenderer.Base, 
        &canvasTexture);

    unsigned int centerX = screenWidth / 2;
    unsigned int centerY = screenHeight / 2;

    BufferTextureRenderer canvasBufferRenderer;
    BufferTextureRenderer_Init(
        canvasTexture.PixelData,
        canvasTexture.Width,
        canvasTexture.Height,
        sizeof(uint16_t),
        &canvasBufferRenderer);

    uint16_t* dateFontTextureBuffer = (uint16_t*)malloc(screenWidth * screenHeight);
    SystemFontTextView dateTextView;
    SystemFontTextView_Init(dateFontTextureBuffer, Colors.Green, Colors.Black, &canvasBufferRenderer.Base, &dateTextView);

    uint16_t* timeFontTextureBuffer = (uint16_t*)malloc(screenWidth * screenHeight);
    SystemFontTextView timeTextView;
    SystemFontTextView_Init(timeFontTextureBuffer, Colors.Green, Colors.Black, &canvasBufferRenderer.Base, &timeTextView);

    Paint_DrawCircle(centerX, centerY, (screenWidth / 2) - 1, YELLOW, false, &canvasTexture);
    while (true) 
    { 
        DateTime time;
        (*dateTimeProvider->GetDateTime)(dateTimeProvider, &time);

        (*dateTimeFormatter->GetDayAndMonth)(dateTimeFormatter, &time, dateTextView.Text);
        Size dateSize = (*dateTextView.Measure)(&dateTextView, screenWidth, screenHeight);
        dateTextView.PositionX = centerX - (dateSize.Width / 2);
        dateTextView.PositionY = centerY - dateSize.Height;
        (*dateTextView.Draw)(&dateTextView);

        (*dateTimeFormatter->GetHoursMinutesSeconds)(dateTimeFormatter, &time, timeTextView.Text);
        Size timeSize = (*timeTextView.Measure)(&timeTextView, screenWidth, screenHeight);
        timeTextView.PositionX = centerX - (timeSize.Width / 2);
        timeTextView.PositionY = centerY + timeSize.Height;
        (*timeTextView.Draw)(&timeTextView);

        (*canvasTexture.Base.Draw)(&canvasTexture.Base, 0, 0);

        (*timer->WaitMilliseconds)(timer, 1000);
    }
}