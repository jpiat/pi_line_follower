
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
