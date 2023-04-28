#ifndef Screen_h    
#define Screen_h    

#include <stdint.h>
#include <stddef.h>

typedef struct Screen
{
    void (*Clear)(struct Screen*, uint16_t color);

    // Sets the viewport being drawn on in relation to the full screen.
    void (*SetViewport)(struct Screen*, uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd);

    // Displays the full image in the buffer.
    // buffer: The raw pixel data to draw.
    // size: The byte size of `buffer`.
    void (*DrawBuffer)(struct Screen*, uint8_t* buffer, size_t size);

    void (*SetBacklightPercentage)(struct Screen*, uint8_t percentage);
    unsigned int (*GetWidth)(struct Screen*);
    unsigned int (*GetHeight)(struct Screen*);
}
Screen;

#endif