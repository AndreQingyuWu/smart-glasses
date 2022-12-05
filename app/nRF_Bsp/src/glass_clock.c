/******************************************************************************

  Copyright (C), 2001-2011, DCN Co., Ltd.

 ******************************************************************************
  File Name     : glass_clock.c
  Version       : Initial Draft
  Author        : x
  Created       : 2019/12/18
  Last Modified :
  Description   : Glass  clock 
  Function List :
              ConvertToUTCTime
              convert_time_to_Second
              get_day_of_week
              get_wall_clock_time
              monthLength
              set_system_clock
              system_clock_init
              update_wall_clock
  History       :
  1.Date        : 2019/12/18
    Author      : x
    Modification: Created file

******************************************************************************/

#include "glass_clock.h"
#include "app_timer.h"

APP_TIMER_DEF(wallClockID);

#define  RTC_TRUE                    (1)
#define  RTC_FALSE                   (0)

/*定义定时器时间间隔*/
#define APP_TIMER_PRESCALER          (0)
#define ONE_MINUTE_INTERVAL          APP_TIMER_TICKS(1000*60, APP_TIMER_PRESCALER)
#define ONE_SECOND_INTERVAL          APP_TIMER_TICKS(1000,    APP_TIMER_PRESCALER)

/*判断年天数*/
#define IsLeapYear(yr)              (!((yr) % 400) || (((yr) % 100) && !((yr) % 4)))
#define YearLength(yr)              (IsLeapYear(yr) ? 366 : 365)

UTCTimeStruct                        Global_Time;
UTCTime                              SecondCountRTC;     
UTCTime                              RTCSTATE;



/*****************************************************************************
 Prototype    : monthLength
 Description  : 获得当月天
 Input        : uint8_t lpyr  
                uint8_t mon   
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/18
    Author       : x
    Modification : Created function

*****************************************************************************/
uint8_t   monthLength( uint8_t lpyr, uint8_t mon )
{
    uint8_t   days = 31;

    if (mon == 1 ) 
    {
        days = ( 28 + lpyr );
    } 
    else 
    {
        if ( mon > 6 ) 
        {
            mon--;
        }
        if ( mon & 1 ) 
        {
            days = 30;
        }
    }
    return ( days );
}
/*****************************************************************************
 Prototype    : convert_time_to_Second
 Description  : 转换时间变为妙
 Input        : time_union_t time  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/18
    Author       : x
    Modification : Created function

*****************************************************************************/
UTCTime convert_time_to_Second(time_union_t time)
{
    uint32_t i = 0;
    UTCTime offset = 0;

    offset += time.time.seconds;
    offset += time.time.minute * 60;
    offset += time.time.hours * 60 * 60;

    uint8_t leapYear = IsLeapYear(time.time.year + 2000);

    offset += DAY * (time.time.day - 1);

    for(i = 0; i < time.time.month - 1; ++i) 
    { 
        offset += monthLength(leapYear,i) * DAY;
    }

    for(i = 0; i< time.time.year ;++i)
    {
        if(IsLeapYear(i + 2000))
        {
            offset += DAY * 366;
        } else {
            offset += DAY * 365;
        }
    }

    return offset;
}
/*****************************************************************************
 Prototype    : ConvertToUTCTime
 Description  : 滴答时钟转换
 Input        : UTCTimeStruct *tm  
                UTCTime secTime    
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/18
    Author       : x
    Modification : Created function

*****************************************************************************/
void ConvertToUTCTime( UTCTimeStruct *tm, UTCTime secTime )
{
     uint32_t day =  secTime % DAY;
     tm->seconds  =  day % 60UL;
     tm->minutes  = (day % 3600UL) / 60UL;
     tm->hour     =  day / 3600UL;

     uint16_t numDays = secTime / DAY;
     tm->year = BEGYEAR;
     
     while ( numDays >= YearLength( tm->year ) )
     {
         numDays -= YearLength( tm->year );
         tm->year++;
     }

     tm->month = 0;
     while ( numDays >= monthLength( IsLeapYear( tm->year ), tm->month ))
     {
         numDays -= monthLength( IsLeapYear( tm->year ), tm->month );
         tm->month++;
     }

     tm->day = numDays;
     
}

/*****************************************************************************
 Prototype    : get_wall_clock_time
 Description  : 获得本地时间
 Input        : void  
 Output       : None
 Return Value : UTCTimeStruct
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/18
    Author       : x
    Modification : Created function

*****************************************************************************/
UTCTimeStruct * get_wall_clock_time(void)
{
    ConvertToUTCTime(&Global_Time,SecondCountRTC);
    Global_Time.month += 1; 
    Global_Time.day   += 1; 
    return &Global_Time;
}
/*****************************************************************************
 Prototype    : update_wall_clock
 Description  : 1S 定时器超时回调
 Input        : void * p_context  
 Output       : None
 Return Value : static
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/18
    Author       : x
    Modification : Created function

*****************************************************************************/
static void update_wall_clock(void * p_context)
{
    (void)p_context;
    
    SecondCountRTC++;
}
/*****************************************************************************
 Prototype    : system_clock_init
 Description  : systimer  clock 初始化
 Input        : void  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/18
    Author       : x
    Modification : Created function

*****************************************************************************/
void system_clock_init(void)
{
    uint32_t err_code;

    Global_Time.year    = 2000;
    Global_Time.month   = 0;
    Global_Time.day     = 0;
    Global_Time.hour    = 0;
    Global_Time.minutes = 0;
    Global_Time.seconds = 0;

    
    err_code = app_timer_create(&wallClockID, APP_TIMER_MODE_REPEATED, update_wall_clock);
    APP_ERROR_CHECK(err_code);
	
    err_code = app_timer_start(wallClockID,ONE_SECOND_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}
/*****************************************************************************
 Prototype    : set_system_clock
 Description  : 设置墙上时钟
 Input        : time_union_t time  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/18
    Author       : x
    Modification : Created function

*****************************************************************************/
void   set_system_clock(time_union_t time)
{
    uint32_t i     = 0;
    UTCTime offset = 0;

    offset += time.time.seconds;
    offset += time.time.minute * 60;
    offset += time.time.hours  * 60 * 60;

    uint8_t leapYear = IsLeapYear(time.time.year + 2000);

    offset += DAY * (time.time.day - 1);

    for(i = 0; i < time.time.month - 1; ++i) 
    { 
        offset += monthLength(leapYear,i) * DAY;
    }

    for(i = 0; i< time.time.year ;++i) 
    {
        if(IsLeapYear(i + 2000))
        {
            offset += DAY * 366;
        } else {
            offset += DAY * 365;
        }
    }

    SecondCountRTC = offset;
    RTCSTATE       = RTC_TRUE;
    
    app_timer_stop(wallClockID);
    app_timer_start(wallClockID, ONE_SECOND_INTERVAL, NULL);
    
}
/*****************************************************************************
 Prototype    : 返回星期
 Description  : 
 Input        : None
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/18
    Author       : x
    Modification : Created function

*****************************************************************************/
DAY_OF_WEEK get_day_of_week(UTCTime secTime)
{
    uint32_t day = secTime / DAY;

    DAY_OF_WEEK today = (DAY_OF_WEEK)(((day %LENGTH_OF_WEEK) + SYSTEM_ORIGIN_DAY_OF_WEEK) %LENGTH_OF_WEEK);

    return today;
}



