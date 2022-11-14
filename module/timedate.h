#ifndef TIME_USER_H
#define TIME_USER_H

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

typedef enum {
	SEC = 1,
	MIN,
	HOUR,
	DAY,
	MONTH,
	YEAR
} eTIMEDATE;

typedef struct {
	int8_t sec;
	int8_t min;
	int8_t hour;
} sTIME;

typedef struct {
	int8_t year;
	int8_t month;
	int8_t day;
} sDATE;

typedef struct {
	int8_t hour;
	int8_t min;
	uint8_t isRepeat;
	uint8_t repeatDay;
	uint8_t en;
} sALARM;

typedef struct {
	uint16_t secF;
	uint16_t minF;
	uint16_t hourF;
} sTIMEFIELD;

void timeSet(sTIME* time, eTIMEDATE mark, int8_t value);
void timeInit(sTIME* time);
eTIMEDATE timeIncSec(sTIME* time);
int16_t timeCompare(sTIME* time1, sTIME* time2);
int16_t getWeekday(int8_t year, int8_t month, int8_t day);
int16_t getDaysFormOneMonth(int16_t year, int16_t month);
void dateInc(sDATE* date, uint8_t s);
void dateDec(sDATE* date, uint8_t s);
void dateFix(sDATE* date);

uint32_t unix_timestamp(uint8_t sec, uint8_t min, uint8_t hrs, uint8_t day, uint8_t mon, uint16_t year);
void utc2local(sTIME* utc_time, sDATE* utc_date, sTIME* timezone, sTIME* local_time, sDATE* local_date);
void local2utc(sTIME* utc_time, sDATE* utc_date, sTIME* timezone, sTIME* local_time, sDATE* local_date);

bool isTimeBetween(sTIME now, sTIME start, sTIME end);

#endif
