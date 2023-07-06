#ifndef UserInput_h
#define UserInput_h

typedef struct UserInput
{
    void (*Plus)(struct UserInput*);
    void (*Minus)(struct UserInput*);
    void (*Select)(struct UserInput*);
    void (*Sleep)(struct UserInput*);
}
UserInput;

#endif