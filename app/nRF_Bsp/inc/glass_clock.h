/******************************************************************************

  Copyright (C), 2001-2011, DCN Co., Ltd.

 ******************************************************************************
  File Name     : glass_clock.h
  Version       : Initial Draft
  Author        : x
  Created       : 2019/12/18
  Last Modified :
  Description   : GLASS  clock
  Function List :
  History       :
  1.Date        : 2019/12/18
    Author      : x
    Modification: Created file

******************************************************************************/

#ifndef NRF_CLOCK_H
#define NRF_CLOCK_H

#include <stdint.h>

#define BEGYEAR                    (2000)  
#define DAY                        (86400UL)   
#define LENGTH_OF_WEEK             (7)
#define SYSTEM_ORIGIN_DAY_OF_WEEK  (Sat)
typedef uint32_t                   UTCTime;

typedef struct
{
    uint16_t   year;     // 2000+
    uint8_t    month;    // 0-11
    uint8_t    day;      // 0-30
    uint8_t    seconds;  // 0-59
    uint8_t    minutes;  // 0-59
    uint8_t    hour;     // 0-23
}UTCTimeStruct;

typedef enum {
    MOn   = 0,
    Tues  = 1,
    Wed   = 2,
    Thur  = 3,
    Fri   = 4,
    Sat   = 5,
    Sun   = 6
    
}DAY_OF_WEEK;

typedef __packed struct
{
   uint32_t year    :6;
   uint32_t month   :4;
   uint32_t day     :5;
   uint32_t hours   :5;
   uint32_t minute  :6;
   uint32_t seconds :6;
   
}time_bit_field_type_t;


typedef __packed  union
{
    uint32_t              data;
    time_bit_field_type_t time;
} time_union_t;


extern   void    system_clock_init(void);
extern   void    set_system_clock(time_union_t time);
extern   UTCTimeStruct *  get_wall_clock_time(void);
#endif

