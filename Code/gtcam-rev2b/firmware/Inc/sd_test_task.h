#ifndef _SD_TEST_H
#define _SD_TEST_H

#include <stdint.h>

typedef struct sd_test_task_state {
    uint32_t target_addr;
} sd_test_task_state_t;

void sd_test_task(void*);

#endif
