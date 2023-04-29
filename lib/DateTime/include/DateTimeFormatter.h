#ifndef DateTimeFormatter_h
#define DateTimeFormatter_h

#include "pico/util/datetime.h"

extern void GetDayOfWeek(datetime_t* time, char* buffer);
extern void GetMonth(datetime_t* time, char* buffer);
extern void GetDayAndMonth(datetime_t* time, char* buffer);
extern void GetHoursMinutesSeconds(datetime_t* time, char* buffer);

#endif