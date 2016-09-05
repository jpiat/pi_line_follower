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

#include <time.h>
#include <stdlib.h>
#include <stdio.h>


#ifndef PROFILE_H
#define PROFILE_H

#ifdef DEBUG
#define DBG(...) printf(__VA_ARGS__)
#else
#define DBG(...)
#endif

struct profile_mem{
	struct timespec * start_time ;
	struct timespec * end_time ;
	struct timespec * diff_time ;
	struct timespec * acc_time ;
};


void init_profile(unsigned int index);
void print_profile_time(char * title, unsigned int index);
void start_profile(unsigned int index);
void end_profile(unsigned int index);


#endif
