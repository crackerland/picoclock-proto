#include "FileStream.h"
#include <stdint.h>
#include <string.h>

static unsigned int SeekRelative(Stream* stream, int byteOffset)
{
    FileStream* this = (FileStream*)stream;
    return fseek(this->File, byteOffset, SEEK_CUR);
}

static unsigned int Seek(Stream* stream, unsigned int byteOffset)
{
    FileStream* this = (FileStream*)stream;
    return fseek(this->File, byteOffset, SEEK_SET);
}

static unsigned int GetPosition(Stream* stream)
{
    FileStream* this = (FileStream*)stream;
    return ftell(this->File);
}

static unsigned int Read(Stream* stream, void* buffer, unsigned long maxReadCount)
{
    FileStream* this = (FileStream*)stream;
    return fread(buffer, sizeof(uint8_t), maxReadCount, this->File);
}

static void Close(Stream* stream)
{
    FileStream* this = (FileStream*)stream;
    fclose(this->File);
}

void FileStream_Init(FILE* file, FileStream* out) 
{
    FileStream stream =
    {
        .Base = 
        {
            .Close = Close,
            .GetPosition = GetPosition,
            .Read = Read,
            .Seek = Seek,
            .SeekRelative = SeekRelative
        },
        .File = file
    };

    memcpy(out, &stream, sizeof(FileStream));
}