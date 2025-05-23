/* See LICENSE of license details. */
#include <errno.h>
#include <sys/time.h>
#include "chip.h"

extern volatile uint32_t SystemCoreClock;
__attribute__((weak)) int _gettimeofday(struct timeval* tp, void* tzp)
{
    uint64_t cycles;

    cycles = __get_rv_cycle();

    tp->tv_sec = cycles / SystemCoreClock;
    tp->tv_usec = (cycles % SystemCoreClock) * 1000000 / SystemCoreClock;
    return 0;
}
