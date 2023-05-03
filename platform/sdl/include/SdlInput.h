#ifndef SdlInput_h
#define SdlInput_h

#include "Input.h"
#include "collections/LinkedList.h"

typedef struct InputHandlerCallback
{
    void (* const Callback)(void* userData, int inputValues);
    void* const UserData;
    const HandlerToken Token;
}
InputHandlerCallback;

typedef struct SdlInputHandler
{
    InputHandler Base;

    // The `InputHandlerCallback`s.
    LinkedList Callbacks;
}
SdlInputHandler;

typedef struct SdlInput 
{
    Input Base;
    SdlInputHandler Handler;

    // Whether the input has been processed or not.
    bool InputPending;

    // The bit flags of the current batch pending processing.
    int CurrentInputs;
}
SdlInput;

extern void SdlInput_Init(SdlInput* out);

#endif