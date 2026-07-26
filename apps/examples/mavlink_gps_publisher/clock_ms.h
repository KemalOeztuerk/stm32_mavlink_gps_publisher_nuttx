/* Milliseconds-since-boot helper, standing in for the original firmware's
 * HAL_GetTick()/osKernelGetTickCount() (both free-running ms tick counts). */
#ifndef CLOCK_MS_H
#define CLOCK_MS_H

#include <time.h>
#include <stdint.h>

static inline uint32_t clock_now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000ULL +
                       (uint64_t)ts.tv_nsec / 1000000ULL);
}

#endif /* CLOCK_MS_H */
