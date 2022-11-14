#include "timedate.h"

static uint16_t isLeapYear(uint16_t year);
static uint8_t dmyInc(sDATE* date, uint8_t s);
static uint8_t dmyDec(sDATE* date, uint8_t s);

void timeSet(sTIME* time, eTIMEDATE mark, int8_t value)
{
	switch (mark) {
		default:
			break;
		case HOUR:
			time->hour = value;
			break;
		case MIN:
			time->min = value;
			break;
		case SEC:
			time->sec = value;
			break;
	}
}

void timeInit(sTIME* time)
{
	timeSet(time, HOUR, 0);
	timeSet(time, MIN, 0);
	timeSet(time, SEC, 0);
}

// for 24 hours
eTIMEDATE timeIncSec(sTIME* time)
{
	int8_t* ptr = &(time->sec);
	(*ptr)++;	//sec++
	if ((*ptr) < 60) {
		return SEC;
	}
	*ptr++ = 0;
	(*ptr)++;
	if ((*ptr) < 60) {
		return MIN;
	}
	*ptr++ = 0;
	(*ptr)++;
	if ((*ptr) < 24) {
		return HOUR;
	}
	*ptr = 0;
	return DAY;
}

// return time1>time2
int16_t timeCompare(sTIME* time1, sTIME* time2)
{
	uint32_t sec1, sec2;
	sec1 = (uint32_t)(time1->hour) * 3600 + time1->min * 60 + time1->sec;
	sec2 = (uint32_t)(time2->hour) * 3600 + time2->min * 60 + time2->sec;
	return sec1 > sec2;
}

static void timezoneTransUtil(sTIME* s_time, sDATE* s_date)
{
	if(s_time->min < 0) {
		s_time->hour -= 1;
		s_time->min += 60;
	}
	if(s_time->hour < 0) {
		dateDec(s_date, DAY);
		s_time->hour += 24;
	}

	if(s_time->min >= 60) {
		s_time->hour += 1;
		s_time->min -= 60;
	}
	if(s_time->hour >= 24) {
		dateInc(s_date, DAY);
		s_time->hour -= 24;
	}
}
void utc2local(sTIME* utc_time, sDATE* utc_date, sTIME* timezone, sTIME* local_time, sDATE* local_date)
{
	memcpy(local_time, utc_time, sizeof(sTIME));
	memcpy(local_date, utc_date, sizeof(sDATE));
	local_time->hour += timezone->hour;
	local_time->min += timezone->min;
	timezoneTransUtil(local_time, local_date);
}

void local2utc(sTIME* utc_time, sDATE* utc_date, sTIME* timezone, sTIME* local_time, sDATE* local_date)
{
	memcpy(utc_time, local_time, sizeof(sTIME));
	memcpy(utc_date, local_date, sizeof(sDATE));
	utc_time->hour -= timezone->hour;
	utc_time->min -= timezone->min;
	timezoneTransUtil(utc_time, utc_date);
}

void dateFix(sDATE* date)
{
	uint8_t maxdays;
	if((*date).month > 12) {
		(*date).month = 12;
	}
	if((*date).year > 99) {
		(*date).year = 99;
	}
	maxdays = getDaysFormOneMonth((*date).year + 2000, (*date).month);
	if((*date).day > maxdays) {
		(*date).day = maxdays;
	}
}

int16_t getWeekday(int8_t year, int8_t month, int8_t day)
{
	static const uint8_t monthPassDay[] = {
		0, 3, 3, 6, 1, 4, 6, 2, 5, 0, 3, 5
	};
	register int8_t days;
	days = monthPassDay[month - 1] + day + (year) + ((year) >> 2);
	if((month <= 2) && ((year & 3) == 0)) {
		days += 6;
	}
	return (days + 6) % 7;
}


uint16_t isLeapYear(uint16_t year)
{
	return (((year % 100 != 0) && (year % 4 == 0)) || (year % 400 == 0));
}

int16_t getDaysFormOneMonth(int16_t year, int16_t month)
{
	switch(month) {
		case 1:
		case 3:
		case 5:
		case 7:
		case 8:
		case 10:
		case 12:
			return 31;
		case 4:
		case 6:
		case 9:
		case 11:
			return 30;
		case 2:
			if(isLeapYear(year)) {
				return 29;
			} else {
				return 28;
			}
		default:
			break;
	}
	return 0;
}


void dateInc(sDATE* date, uint8_t s)
{
	if(s != 0) {
		dateInc(date, dmyInc(date, s));
	}
}

void dateDec(sDATE* date, uint8_t s)
{
	if(s != 0) {
		dateDec(date, dmyDec(date, s));
	}
}

uint8_t dmyInc(sDATE* date, uint8_t s)
{
	uint8_t _temp;
	switch(s) {
		case DAY:
			_temp = getDaysFormOneMonth(((*date).year < 100) ? ((*date).year + 2000) : ((*date).year), (*date).month);
			if((*date).day == _temp) {
				(*date).day = 1;
				return MONTH;
			} else {
				(*date).day++;
			}
			break;
		case MONTH:
			if((*date).month == 12) {
				(*date).month = 1;
				return YEAR;
			} else {
				(*date).month++;
			}
			break;
		case YEAR:
			if((*date).year >= 99) {
				(*date).year = 0;
			} else {
				(*date).year++;
			}
			break;
	}
	_temp = getDaysFormOneMonth(((*date).year < 100) ? ((*date).year + 2000) : ((*date).year), (*date).month);
	if((*date).day > _temp) {
		(*date).day = _temp;
	}
	return 0;
}

uint8_t dmyDec(sDATE* date, uint8_t s)
{
	uint8_t _temp;
	switch(s) {
		case DAY:
			_temp = getDaysFormOneMonth(((*date).year < 100) ? ((*date).year + 2000) : ((*date).year), (*date).month);
			if((*date).day == 1) {
				(*date).day = _temp;
				return MONTH;
			} else {
				(*date).day--;
			}
			break;
		case MONTH:
			if((*date).month == 1) {
				(*date).month = 12;
				return YEAR;
			} else {
				(*date).month--;
			}
			break;
		case YEAR:
			if((*date).year <= 0) {
				(*date).year = 99;
			} else {
				(*date).year--;
			}
			break;
	}
	_temp = getDaysFormOneMonth(((*date).year < 100) ? ((*date).year + 2000) : ((*date).year), (*date).month);
	if((*date).day > _temp) {
		(*date).day = _temp;
	}
	return 0;
}

#define SEC_P_DAY (86400)
#define SEC_P_NORMAL_YEAR (31536000)
#define SEC_P_LEAP_YEAR (31622400)
#define UNIX_2000_UTC (946684800)
uint32_t unix_timestamp(uint8_t sec, uint8_t min, uint8_t hrs, uint8_t day, uint8_t mon, uint16_t year)
{
	uint16_t y = year - 1;		// year var
	uint32_t unix = UNIX_2000_UTC;
	uint16_t d = 0;			// day var
	uint8_t m = mon - 1;			// month var

	while(y >= 2000) {
		if(isLeapYear(y)) {
			unix += SEC_P_LEAP_YEAR;
		} else {
			unix += SEC_P_NORMAL_YEAR;
		}
		y -= 1;
	}

	while(m > 0) {
		d += getDaysFormOneMonth(year, m);
		m -= 1;
	}
	unix += (d + day - 1) * SEC_P_DAY;
	unix += hrs * 3600 + min * 60 + sec;
	return unix;
}

// is now between start and end.
bool isTimeBetween(sTIME now, sTIME start, sTIME end)
{
	uint32_t t0 = start.hour * 3600 + start.min * 60 + start.sec;
	uint32_t t1 = end.hour * 3600 + end.min * 60 + end.sec;
	uint32_t tn = now.hour * 3600 + now.min * 60 + now.sec;
	if(t0 == t1) {
		return false;
	}
	if(t0 < t1) {
		return ((tn >= t0) && (tn <= t1));
	} else {
		return ((tn >= t1) || (tn <= t0));
	}
}
