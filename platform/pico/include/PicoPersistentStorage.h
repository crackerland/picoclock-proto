#ifndef PicoPersistentStorage_h
#define PicoPersistentStorage_h

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#define PERSISTENT_STORGE_KEY_SIZE 16

/// @param key The null-terminated key of the content to load. Must match the key used when saved.
/// @return Whether saved state was found and `out` is valid. 
extern bool LoadFlashState(char key[PERSISTENT_STORGE_KEY_SIZE], uint8_t* out, size_t length);

/// @param key An arbitrary null-terminated key for the data being stored. This will be used when loading later. 
/// @param data The raw data to save.
/// @param size The number of bytes in `data` to save.
extern void SaveFlashState(char key[PERSISTENT_STORGE_KEY_SIZE], uint8_t* data, size_t size);

extern void EraseAll();

#endif