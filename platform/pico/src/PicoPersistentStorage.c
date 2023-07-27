#include "hardware/flash.h"

#define SAVE_STATE_CONTENT_SIZE (FLASH_PAGE_SIZE - 5)
typedef struct SaveState
{
    char Name[5];
    uint8_t Content[SAVE_STATE_CONTENT_SIZE];
}
SaveState;

// Allocate a single sector (which is the minimum) at the end of the flash memory
// for user data. Program data is at the beginning, so this will not conflict.
#define FLASH_USER_DATA_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
const uint8_t* flashContents = (const uint8_t*) (XIP_BASE + FLASH_USER_DATA_OFFSET);
static SaveState* LoadFlashState()
{
    return (SaveState*)flashContents;
}

static void SaveFlashState(uint8_t* data, size_t size)
{
    SaveState state = 
    {
        .Name = "SAVE"
    };

    memcpy(&state.Content, data, size);

    // A whole number of sectors must be erased at a time.
    flash_range_erase(FLASH_USER_DATA_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_USER_DATA_OFFSET, (uint8_t*)&state, FLASH_PAGE_SIZE);
}