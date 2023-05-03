#ifndef Input_h
#define Input_h

typedef enum InputState
{
    InputState_Down,
    InputState_Up,
    InputState_Held
}
InputState;

typedef enum InputValue
{
    InputValue_Up = 1,
    InputValue_Down = 1 << 1,
    InputValue_Right = 1 << 2,
    InputValue_Left = 1 << 3,
    InputValue_A = 1 << 5,
    InputValue_B = 1 << 6,
    InputValue_Start = 1 << 7,
    InputValue_Select = 1 << 8
}
InputValue;

typedef unsigned int HandlerToken;
typedef struct InputHandler
{
    HandlerToken (*AddHandler)(
        struct InputHandler*, 
        void (*callback)(void* userData, int inputValues), 
        void* userData);

    void (*RemoveHandler)(struct InputHandler*, HandlerToken token);
}
InputHandler;

typedef struct Input
{
    void (*Up)(struct Input*, InputState state);
    void (*Down)(struct Input*, InputState state);
    void (*Left)(struct Input*, InputState state);
    void (*Right)(struct Input*, InputState state);
    void (*A)(struct Input*, InputState state);
    void (*B)(struct Input*, InputState state);
    void (*Start)(struct Input*, InputState state);
    void (*Select)(struct Input*, InputState state);

    // Processes the inputs received up to the point that this is called.
    void (*ProcessBatch)(struct Input*);
}
Input;

#endif