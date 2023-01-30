#ifndef _RUNTIME_STATS_TASK_H
#define _RUNTIME_STATS_TASK_H

#include <stdint.h>

typedef struct taskname_uptime_pair {
    const char* taskname;
    uint32_t uptime;
} taskname_uptime_pair_t;

void runtime_stats_task(void* arg);

#endif
