#ifndef Battery_h
#define Battery_h

typedef struct Battery
{ 
    float const (*Read)(struct Battery*);
} 
Battery;

#endif