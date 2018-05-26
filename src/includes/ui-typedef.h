#ifndef _UI_TYPEDEF_DEF_H_
#define _UI_TYPEDEF_DEF_H_

#include <stdint.h>
#include <stdbool.h>

#include "am_type.h"

#pragma pack(1) // 1 bytes对齐
typedef struct
{
    U8 hour;
    U8 min;
    U8 sec;
    //公历日月年周
    U16 w_year;
    U8  w_month;
    U8  w_date;
    U8  week;
}tm;
#pragma pack ()
typedef struct ui_schedule_union_t
{
    U8 hour;
    U8 minute;
    U8 enable;
}ui_schedule_union_t;


#endif /* _UI_TYPEDEF_DEF_H_ */

