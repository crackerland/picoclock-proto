#include "DateTimeFormatter.h"
#include <stdio.h>
#include <string.h>

static const char* DaysOfWeek[] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" }; 

static const char* Months[] = 
{ 
    "January", 
    "February", 
    "March", 
    "April",
    "May",
    "June",
    "July",
    "August",
    "September",
    "October",
    "November",
    "December"
}; 

void GetDayOfWeek(datetime_t* time, char* buffer)
{
    const char* text = DaysOfWeek[time->dotw];
    memcpy(buffer, text, strlen(text));
}

void GetMonth(datetime_t* time, char* buffer)
{
    const char* text = Months[time->dotw];
    memcpy(buffer, text, strlen(text));
}

void GetDayAndMonth(datetime_t* time, char* buffer)
{
    char day[16];
    char month[16];
    GetDayOfWeek(time, day);
    GetMonth(time, month);

    sprintf(buffer, "%s, %s %d", day, month, time->day);
}

void GetHoursMinutesSeconds(datetime_t* time, char* buffer)
{
    sprintf(buffer, "%d:%d:%d", time->hour % 12, time->min, time->sec);
}