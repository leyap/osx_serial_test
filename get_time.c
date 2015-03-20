//#include <sys/time.h>
#include <time.h>
#include "get_time.h"

//int gettimeofday (struct timeval *tv, struct timezone *tz);

unsigned long get_time() {
	unsigned long cost_time = 0;
	struct timeval t_start;
	gettimeofday (&t_start, NULL);
	return ((unsigned long)t_start.tv_sec)*1000+(unsigned long)t_start.tv_usec/1000;
}

