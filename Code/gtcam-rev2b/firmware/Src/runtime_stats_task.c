/**
 * This task takes a snapshot of the freertos system state about once a second to get information
 * about all the tasks in the system, including the total uptime of each task.
 *
 * As a convenience, it also creates an array of pairs of (taskname, uptime in microsec).
 */

#include "runtime_stats_task.h"
#include "cmsis_os.h"

// redefinition with static scope so that GDB can see it.
volatile TaskStatus_t* task_statuses;
volatile taskname_uptime_pair_t* task_uptimes;

void runtime_stats_task(void* arg)
{
    // we assume that the number of tasks doesn't change as time goes on, so we can allocate an
    // array on the stack when this task starts.
    uint32_t total_time;
    const UBaseType_t num_tasks = uxTaskGetNumberOfTasks();
    TaskStatus_t task_status_array[num_tasks];
    taskname_uptime_pair_t task_uptime_array[num_tasks];
    task_statuses = task_status_array;
    task_uptimes = task_uptime_array;

    while(1) {
        uxTaskGetSystemState(task_status_array, num_tasks, &total_time);

        for (int i = 0; i < num_tasks; i++) {
            task_uptime_array[i].taskname = task_status_array[i].pcTaskName;
            task_uptime_array[i].uptime = task_status_array[i].ulRunTimeCounter;
        }

        vTaskDelay(399);
    }
}
