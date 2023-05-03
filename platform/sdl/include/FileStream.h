#ifndef FileStream_h
#define FileStream_h

#include "Stream.h"
#include <stdio.h>

typedef struct FileStream 
{
    Stream Base;
    FILE* File;
}
FileStream;

extern void FileStream_Init(FILE* file, FileStream* out); 

#endif