#include "PicoPersistentStorage.h"
#include "hardware/flash.h"

#define SAVE_STATE_CONTENT_SIZE (FLASH_PAGE_SIZE - 5)
typedef struct SaveState
{
    char Name[PERSISTENT_STORGE_KEY_SIZE];
    uint8_t Content[SAVE_STATE_CONTENT_SIZE];
}
SaveState;

// Allocate a single sector (which is the minimum) at the end of the flash memory
// for user data. Program data is at the beginning, so this will not conflict.
#define FLASH_USER_DATA_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
const uint8_t* flashContents = (const uint8_t*) (XIP_BASE + FLASH_USER_DATA_OFFSET);
bool LoadFlashState(char key[PERSISTENT_STORGE_KEY_SIZE], uint8_t* out, size_t length)
{
    SaveState* state = (SaveState*)flashContents;
    if (strcmp(state->Name, key))
    {
        return false;
    }

    memcpy(out, state->Content, length);
    return true;
}

void SaveFlashState(char key[PERSISTENT_STORGE_KEY_SIZE], uint8_t* data, size_t size)
{
    SaveState state = { };
    memcpy(state.Name, key, PERSISTENT_STORGE_KEY_SIZE);
    state.Name[PERSISTENT_STORGE_KEY_SIZE] = '\0';

    memcpy(&state.Content, data, size);

    EraseAll();
    flash_range_program(FLASH_USER_DATA_OFFSET, (uint8_t*)&state, FLASH_PAGE_SIZE);
}

void EraseAll()
{
    // A whole number of sectors must be erased at a time.
    flash_range_erase(FLASH_USER_DATA_OFFSET, FLASH_SECTOR_SIZE);
}