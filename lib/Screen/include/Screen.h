#ifndef Screen_h    
#define Screen_h    

#include <stdint.h>
#include <stddef.h>

typedef struct Screen
{
    void (*Clear)(struct Screen*, uint16_t color);

    // Displays the full image in the buffer.
    void (*DrawBuffer)(struct Screen*, uint16_t* buffer);

// function:	Sets the start position and size of the display area
// parameter:
// 		Xstart 	:   X direction Start coordinates
// 		Ystart  :   Y direction Start coordinates
// 		Xend    :   X direction end coordinates
// 		Yend    :   Y direction end coordinates
    void (*SetViewport)(struct Screen*, uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd);

    void (*DisplayWindows)(
        struct Screen*, 
        unsigned int xStart, 
        unsigned int yStart, 
        unsigned int xEnd, 
        unsigned int yEnd, 
        uint8_t *image,
        size_t pixelSize);

    void (*DrawPoint)(struct Screen*, uint16_t x, uint16_t y, uint16_t color);
    void (*SetBacklightPercentage)(struct Screen*, uint8_t percentage);
    unsigned int (*GetWidth)(struct Screen*);
    unsigned int (*GetHeight)(struct Screen*);
}
Screen;

#endif