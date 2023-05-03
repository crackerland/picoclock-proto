#include "SdlInput.h"
#include <string.h>
#include <stdlib.h>

static inline void SetValue(Input* input, InputState state, InputValue value)
{
    SdlInput* this = (SdlInput*)input;
    switch (state)
    {
        case InputState_Down:
            this->CurrentInputs |= value;
            break;

        case InputState_Up:
            this->CurrentInputs &= ~value;
            break;
    }

    this->InputPending = true;
}

static void Up(Input* input, InputState state)
{
    SetValue(input, state, InputValue_Up);
}

static void Down(Input* input, InputState state)
{
    SetValue(input, state, InputValue_Down);
}

static void Left(Input* input, InputState state)
{
    SetValue(input, state, InputValue_Left);
}

static void Right(Input* input, InputState state)
{
    SetValue(input, state, InputValue_Right);
}

static void A(Input* input, InputState state)
{
    SetValue(input, state, InputValue_A);
}

static void B(Input* input, InputState state)
{
    SetValue(input, state, InputValue_B);
}

static void Start(Input* input, InputState state)
{
    SetValue(input, state, InputValue_Start);
}

static void Select(Input* input, InputState state)
{
    SetValue(input, state, InputValue_Select);
}

static void ProcessBatch(Input* input)
{
    SdlInput* this = (SdlInput*)input;

    if (!this->InputPending)
    {
        // Nothing to process.
        return;
    }

    this->InputPending = false;

    Iterable* iterable = (*this->Handler.Callbacks.Base.GetIterable)(&this->Handler.Callbacks.Base);
    Iterator* next = (iterable->Start)(iterable);
    do
    {
        InputHandlerCallback* handlerCallback = (*next->GetValue)(next);
        (*handlerCallback->Callback)(handlerCallback->UserData, this->CurrentInputs);
    } 
    while ((next = (*next->GetNext)(next)));
}

static HandlerToken AddHandler(
    InputHandler* handler, 
    void (*callback)(void* userData, int inputValues), 
    void* userData)
{
    SdlInputHandler* this = (SdlInputHandler*)handler;

    InputHandlerCallback handlerCallback = 
    {
        .Callback = callback,
        .UserData = userData,
        .Token = (*this->Callbacks.Base.GetCount)(&this->Callbacks.Base)
    };

    InputHandlerCallback* output = malloc(sizeof(InputHandlerCallback));
    memcpy(output, &handlerCallback, sizeof(InputHandlerCallback));
    (*this->Callbacks.Base.Add)(&this->Callbacks.Base, output);

    return handlerCallback.Token;
}

static void RemoveHandler(InputHandler* handler, HandlerToken token)
{
    SdlInputHandler* this = (SdlInputHandler*)handler;
    Iterable* iterable = (*this->Callbacks.Base.GetIterable)(&this->Callbacks.Base);
    Iterator* next = (iterable->Start)(iterable);
    do
    {
        InputHandlerCallback* handlerCallback = (*next->GetValue)(next);
        if (handlerCallback->Token == token)
        {
            (*this->Callbacks.Base.Remove)(&this->Callbacks.Base, handlerCallback);
            free(handlerCallback);
            return;
        }
    } 
    while ((next = (*next->GetNext)(next)));
}

void SdlInput_Init(SdlInput* out)
{
    SdlInput input = 
    {
        .Base = 
        {
            .Up = Up,
            .Down = Down,
            .Left = Left,
            .Right = Right,
            .A = A,
            .B = B,
            .Start = Start,
            .Select = Select,
            .ProcessBatch = ProcessBatch
        },
        .Handler = 
        {
            .Base =
            {
                .AddHandler = AddHandler,
                .RemoveHandler = RemoveHandler
            }
        }
    };

    LinkedList_Init(&input.Handler.Callbacks);
    memcpy(out, &input, sizeof(SdlInput));
}