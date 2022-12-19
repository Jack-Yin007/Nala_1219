/*******************************************************************************
* File Name          : rtcclock.h
* Author             : Yangjie Gu
* Description        : This file provides all the rtcclock functions.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _RTC_CLOCK_H
#define _RTC_CLOCK_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Defines -------------------------------------------------------------------*/
#define LEAP_YEAR 366
#define NORM_YEAR 365
#define BASE_YEAR 1980
#define LEAP_LOOP (366 + (3 * 365))
#define BASE_OFFSET 432000

typedef struct
{
    uint32_t year;
    uint32_t month;
    uint32_t day;
    uint32_t hour;
    uint32_t minute;
    uint32_t second;
} data_time_table_t;

extern data_time_table_t SecondsToTimeTable(uint32_t seconds);
extern uint32_t TimeTableToSeconds(data_time_table_t timeTable);

#ifdef __cplusplus
}
#endif

#endif /* _RTC_CLOCK_H */
