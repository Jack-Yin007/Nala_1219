/* Includes ------------------------------------------------------------------*/
#include "rtcclock.h"

/* Private define ------------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Public variables ----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const uint8_t norm_month_table[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

/* Function prototypes -------------------------------------------------------*/
uint32_t Divide(uint32_t Second, uint32_t value, uint32_t *returnValue)
{
    *returnValue = Second % value;

    return (Second / value);
}

int8_t GetMonthFromDays(uint32_t days, uint32_t year, uint32_t *returnDays)
{
    int32_t i = 0;
    uint32_t totalday = 0;

    for (i = 0; i < 12; i++)
    {
        if (days < totalday)
        {
            break;
        }
        if (((year & 0x3) == 0) && (i == 1))
        {
            totalday += 29;
        }
        else
        {
            totalday += norm_month_table[i];
        }
    }

    if (((year & 0x3) == 0) && (i == 2))
    {
        *returnDays = days - totalday + 29 + 1;
    }
    else
    {
        *returnDays = days - totalday + norm_month_table[i - 1] + 1;
    }

    return i;
}

data_time_table_t SecondsToTimeTable(uint32_t seconds)
{
    data_time_table_t timeTable;
    uint32_t days;

    seconds += BASE_OFFSET;

    //get seconds, minute and hour;
    seconds = Divide(seconds, 60, &timeTable.second);
    seconds = Divide(seconds, 60, &timeTable.minute);
    seconds = Divide(seconds, 24, &timeTable.hour);

    //count how many leap_loop be included;
    uint32_t leap = Divide(seconds, LEAP_LOOP, &days);
    timeTable.year = BASE_YEAR + 4 * leap;

    //get surplus days to determine the appointed year;
    if (days < 366)
    {
    }
    else if (days < (366 + 365))
    {
        timeTable.year += 1;
        days -= (366);
    }
    else if (days < (366 + 365 * 2))
    {
        timeTable.year += 2;
        days -= (366 + 365);
    }
    else if (days < (366 + 365 * 3))
    {
        timeTable.year += 3;
        days -= (366 + 365 * 2);
    }

    timeTable.month = GetMonthFromDays(days, timeTable.year, &timeTable.day);
    return timeTable;
}

uint32_t TimeTableToSeconds(data_time_table_t timeTable)
{
    uint32_t seconds = 0;
    int32_t i;

    //get total days from 1980 not included nowyear
    for (i = BASE_YEAR; i < timeTable.year; i++)
    {
        //if %4 != 0;
        if ((i & 0x3) != 0)
        {
            seconds += NORM_YEAR;
        }
        else
        {
            seconds += LEAP_YEAR;
        }
    }
    //get nowyears total days not included this month;
    for (i = 1; i < timeTable.month; i++)
    {
        //if leap year and 2th month;
        if (((timeTable.year & 0x3) == 0) && (i == 2))
            seconds += 29;
        else
            seconds += norm_month_table[i - 1];
    }
    //get this month's days;
    seconds += timeTable.day - 1;
    seconds = seconds * 24 + timeTable.hour;
    seconds = seconds * 60 + timeTable.minute;
    seconds = seconds * 60 + timeTable.second;

    seconds -= BASE_OFFSET;
    return seconds;
}
