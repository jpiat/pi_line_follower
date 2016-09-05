/* * eSLAM - eSLAM is an embedded implementation for a vision-based Simultaneous
 * Localization And Mapping using extended Kalman Filter
 * Copyright (C) 2014 LAAS/CNRS
 *
 * This file is part of eSLAM.
 *
 * eSLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * eSLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with eSLAM.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "profile.h"

struct profile_mem prof_array[20];
unsigned int initialized = 0 ;
#define IS_INIT(x) ((initialized >> x) & 0x01)
#define SET_INIT(x) initialized |= (1 << x)
struct timespec cpu_time;

int timespec_subtract(struct timespec * result, struct timespec * x,
		struct timespec * y) {
	/* Perform the carry for the later subtraction by updating y. */
	if (x->tv_nsec < y->tv_nsec) {
		double nbsec = (y->tv_nsec - x->tv_nsec) / 1000000000.0;
		y->tv_nsec -= nbsec * 1000000000;
		y->tv_sec += nbsec;
	}
	if (x->tv_nsec - y->tv_nsec > 1000000000) {
		double nbsec = (x->tv_nsec - y->tv_nsec) / 1000000000.0;
		y->tv_nsec += nbsec * 1000000000.0;
		y->tv_sec -= nbsec;
	}

	/* Compute the time remaining to wait.
	 tv_usec is certainly positive. */
	result->tv_sec = x->tv_sec - y->tv_sec;
	result->tv_nsec = x->tv_nsec - y->tv_nsec;

	/* Return 1 if result is negative. */
	return x->tv_sec < y->tv_sec;
}

void init_profile(unsigned int index) {
	prof_array[index].acc_time = (struct timespec *) malloc(
			sizeof(struct timespec));
	memset(prof_array[index].acc_time, 0, sizeof(struct timespec));
	prof_array[index].start_time = (struct timespec *) malloc(
			sizeof(struct timespec));
	prof_array[index].end_time = (struct timespec *) malloc(
			sizeof(struct timespec));
	prof_array[index].diff_time = (struct timespec *) malloc(
			sizeof(struct timespec));
	SET_INIT(index);
}

void print_profile_time(char * title, unsigned int index) {
	DBG("%s :  %lu s, %lu ns \n", title, prof_array[index].acc_time->tv_sec, prof_array[index].acc_time->tv_nsec);
}

void start_profile(unsigned int index) {
#ifdef DEBUG
	if(IS_INIT(index) == 0) init_profile(index);
	clock_gettime(CLOCK_REALTIME, prof_array[index].start_time);
#endif
}

void end_profile(unsigned int index) {
#ifdef DEBUG
	if(IS_INIT(index) == 0){
		init_profile(index);
		return ;
	}
	clock_gettime(CLOCK_REALTIME, prof_array[index].end_time);
	timespec_subtract(prof_array[index].diff_time, prof_array[index].end_time,
			prof_array[index].start_time);
	prof_array[index].acc_time->tv_sec += prof_array[index].diff_time->tv_sec;
	prof_array[index].acc_time->tv_nsec += prof_array[index].diff_time->tv_nsec;
	if(prof_array[index].acc_time->tv_nsec > 1000000000) {
		prof_array[index].acc_time->tv_nsec -= 1000000000;
		prof_array[index].acc_time->tv_sec += 1;
	}
#endif
}

