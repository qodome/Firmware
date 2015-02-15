#include <stdint.h>
#include "app_util.h"
#include "UTCtime_convert.h"

#define	BEGYEAR	        2015     // UTC started at 00:00:00 January 1, 2015
#define	DAY             86400UL  // 24 hours * 60 minutes * 60 seconds
#define	YearLength(yr)	(IsLeapYear(yr) ? 366 : 365)
#define	IsLeapYear(yr)	(!((yr) % 400) || (((yr) % 100) && !((yr) % 4)))

static uint8_t monthLength(uint8_t lpyr, uint8_t mon);

void osal_ConvertUTCTime(ble_date_time_t *tm, uint32_t secTime)
{
	// calculate the time less than a day - hours, minutes, seconds
	{
		uint32_t day = secTime % DAY;
		tm->seconds = day % 60UL;
		tm->minutes = (day % 3600UL) / 60UL;
		tm->hours = day / 3600UL;
	}

	// Fill in the calendar - day, month, year
	{
		uint16_t numDays = secTime / DAY;
		tm->year = BEGYEAR;
		while (numDays >= YearLength(tm->year))
		{
			numDays -= YearLength(tm->year);
			tm->year++;
		}

		tm->month = 0;
		while (numDays >= monthLength(IsLeapYear(tm->year), tm->month))
		{
			numDays -= monthLength(IsLeapYear(tm->year), tm->month);
			tm->month++;
		}

		tm->day = numDays;
	}

	tm->month++;
	tm->day++;
}


/*********************************************************************
 * @fn      monthLength
 *
 * @param   lpyr - 1 for leap year, 0 if not
 *
 * @param   mon - 0 - 11 (jan - dec)
 *
 * @return  number of days in specified month
 */
static uint8_t monthLength(uint8_t lpyr, uint8_t mon)
{
	uint8_t days = 31;

	if (mon == 1) // feb
	{
		days = (28 + lpyr);
	}
	else
	{
		if (mon > 6) // aug-dec
		{
			mon--;
		}

		if (mon & 1)
		{
			days = 30;
		}
	}

	return (days);
}

/*********************************************************************
 * @fn      osal_ConvertUTCSecs
 *
 * @brief   Converts a UTCTimeStruct to UTCTime
 *
 * @param   tm - pointer to provided struct
 *
 * @return  number of seconds since 00:00:00 on 01/01/2000 (UTC)
 */
uint32_t osal_ConvertUTCSecs(ble_date_time_t *tm)
{
	uint32_t seconds;

	tm->month--;
	tm->day--;

	/* Seconds for the partial day */
	seconds = (((tm->hours * 60UL) + tm->minutes) * 60UL) + tm->seconds;

	/* Account for previous complete days */
	{
		/* Start with complete days in current month */
		uint16_t days = tm->day;

		/* Next, complete months in current year */
		{
			int8_t month = tm->month;
			while (--month >= 0)
			{
				days += monthLength(IsLeapYear(tm->year), month);
			}
		}

		/* Next, complete years before current year */
		{
			uint16_t year = tm->year;
			while (--year >= BEGYEAR)
			{
				days += YearLength(year);
			}
		}

		/* Add total seconds before partial day */
		seconds += (days * DAY);
	}

	return (seconds);
}
