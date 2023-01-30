#include "main.h"
#include "sd_driver_task.h"
#include "cmsis_os.h"

extern uint32_t sd_driver_task_most_recent_write_timestamp;
extern uint32_t mlx_read_task_most_recent_frame_timestamp;
extern uint32_t camera_read_task_most_recent_frame_timestamp;

extern uint32_t sd_driver_task_queue_fail_count;

const uint32_t sd_timeout = 100;
const uint32_t mlx_timeout = 100;
const uint32_t camera_timeout = 100;

void system_health_indicator_task(void* param)
{
    while (1) {
        volatile uint32_t sd_stale_time = (uwTick - sd_driver_task_most_recent_write_timestamp);
        volatile uint32_t mlx_stale_time = (uwTick - mlx_read_task_most_recent_frame_timestamp);
        volatile uint32_t camera_stale_time = (uwTick - camera_read_task_most_recent_frame_timestamp);
        const int healthy = ((sd_stale_time < sd_timeout) &&
                             (mlx_stale_time < mlx_timeout) &&
                             (camera_stale_time < camera_timeout) &&
                             (sd_driver_task_queue_fail_count == 0));
        if (healthy) {
            // blink LED at ~4Hz if the system is healthy.
            if ((uwTick % 0x100) < 0x80) {
                LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_3);
            } else {
                LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_3);
            }
        } else {
            // shut the LED off
            LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_3);
        }

        vTaskDelay(3);
    }
}
