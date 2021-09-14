/* Some of the source code in this file came from "linux/fs/fat/misc.c".  */
/*
 *  linux/fs/fat/misc.c
 *
 *  Written 1992,1993 by Werner Almesberger
 *  22/11/2000 - Fixed fat_date_unix2dos for dates earlier than 01/01/1980
 *         and date_dos2unix for date==0 by Igor Zhbanov(bsg@uniyar.ac.ru)
 */

/*
 *  Copyright (C) 2012-2013 Samsung Electronics Co., Ltd.
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/************************************************************************/
/*                                                                      */
/*  PROJECT : exFAT & FAT12/16/32 File System                           */
/*  FILE    : exfat_oal.c                                               */
/*  PURPOSE : exFAT OS Adaptation Layer                                 */
/*            (Semaphore Functions & Real-Time Clock Functions)         */
/*                                                                      */
/*----------------------------------------------------------------------*/
/*  NOTES                                                               */
/*                                                                      */
/*----------------------------------------------------------------------*/
/*  REVISION HISTORY (Ver 0.9)                                          */
/*                                                                      */
/*  - 2010.11.15 [Joosun Hahn] : first writing                          */
/*                                                                      */
/************************************************************************/

#include <linux/semaphore.h>
#include <linux/time.h>
#ifndef _TIME_T
typedef time64_t    time_t;
#endif

#include "exfat_api.h"
#include "exfat_oal.h"

/*======================================================================*/
/*                                                                      */
/*            SEMAPHORE FUNCTIONS                                       */
/*                                                                      */
/*======================================================================*/

DEFINE_SEMAPHORE(z_sem);

s32 sm_init(struct semaphore *sm)
{
	sema_init(sm, 1);
	return 0;
} /* end of sm_init */

s32 sm_P(struct semaphore *sm)
{
	down(sm);
	return 0;
} /* end of sm_P */

void sm_V(struct semaphore *sm)
{
	up(sm);
} /* end of sm_V */


/*======================================================================*/
/*                                                                      */
/*            REAL-TIME CLOCK FUNCTIONS                                 */
/*                                                                      */
/*======================================================================*/

extern struct timezone sys_tz;

/*
 * The epoch of FAT timestamp is 1980.
 *     :  bits  : value
 * date:  0 -  4: day    (1 -  31)
 * date:  5 -  8: month  (1 -  12)
 * date:  9 - 15: year   (0 - 127) from 1980
 * time:  0 -  4: sec    (0 -  29) 2sec counts
 * time:  5 - 10: min    (0 -  59)
 * time: 11 - 15: hour   (0 -  23)
 */
#define UNIX_SECS_1980   315532800L

#if BITS_PER_LONG == 64
#define UNIX_SECS_2108   4354819200L
#endif
/* days between 1.1.70 and 1.1.80 (2 leap days) */
#define DAYS_DELTA_DECADE	(365 * 10 + 2)
/* 120 (2100 - 1980) isn't leap year */
#define NO_LEAP_YEAR_2100    (120)
#define IS_LEAP_YEAR(y)  (!((y) & 3) && (y) != NO_LEAP_YEAR_2100)

#define SECS_PER_MIN     (60)
#define SECS_PER_HOUR    (60 * SECS_PER_MIN)
#define SECS_PER_DAY     (24 * SECS_PER_HOUR)

TIMESTAMP_T *tm_current(TIMESTAMP_T *tp)
{
	struct timespec64 ts;
	time_t second;
	struct tm tm;

	ktime_get_real_ts64(&ts);

	second = ts.tv_sec;
	second -= sys_tz.tz_minuteswest * SECS_PER_MIN;

	/* Jan 1 GMT 00:00:00 1980. But what about another time zone? */
	if (second < UNIX_SECS_1980) {
		tp->sec  = 0;
		tp->min  = 0;
		tp->hour = 0;
		tp->day  = 1;
		tp->mon  = 1;
		tp->year = 0;
		return tp;
	}
#if BITS_PER_LONG == 64
	if (second >= UNIX_SECS_2108) {
		tp->sec  = 59;
		tp->min  = 59;
		tp->hour = 23;
		tp->day  = 31;
		tp->mon  = 12;
		tp->year = 127;
		return tp;
	}
#endif

	time64_to_tm(second, 0, &tm);

	tp->sec  = tm.tm_sec;
	tp->min  = tm.tm_min;
	tp->hour = tm.tm_hour;
	tp->day  = tm.tm_mday;
	tp->mon  = tm.tm_mon + 1;
	tp->year = tm.tm_year - 80;

	return tp;
} /* end of tm_current */
