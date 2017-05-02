#include <stdlib.h>
#include <sys/time.h>

#include "util.h"

int64_t utime_now (void)
{
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}
