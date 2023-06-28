#include "Painter.h"

static void DrawPoint16(uint16_t color, unsigned int x, unsigned int y, Texture16* texture)
{
    texture->PixelData[y * texture->Base.TextureBuffer.Width + x] = color;
}

static void DrawPoint(Painter* painter, Color color, unsigned int x, unsigned int y, Texture16* texture)
{
    DrawPoint16((*painter->ColorConverter->Convert565)(painter->ColorConverter, color), x, y, texture);
}

static void DrawCircle(
    Painter* painter, 
    uint16_t xCenter, 
    uint16_t yCenter, 
    uint16_t radius, 
    Color colorIn, 
    bool drawFilled,
    Texture16* texture)
{
    uint16_t color = (*painter->ColorConverter->Convert565)(painter->ColorConverter, colorIn);

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

void Painter_Init(ColorConverter* colorConverter, Painter* out)
{
    out->ColorConverter = colorConverter;
    out->DrawCircle = DrawCircle;
    out->DrawPoint = DrawPoint;
}