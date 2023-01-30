#include "annotator_task.h"
#include "main.h"
#include "sd_driver_task.h"

#define BLOCKS_PER_ENTRY (1)

#define ANNOTATION_SD_OFFSET (0x00b80000ul)

static uint8_t buf[512];

static void my_itoa_padded(char* s, uint32_t n, int len)
{
    for (int i = (len - 1); i >= 0; i--) {
        s[i] = n % 10 + '0';
        n /= 10;
    }
}

void annotator_task(void* arg)
{
    sd_driver_rw_request_t sd_write_request;
    sd_write_request.direction = SD_DRIVER_DIRECTION_WRITE;
    sd_write_request.nblocks = 1;
    sd_write_request.buf = buf;

    int annotation_number = 0;

    int32_t sw1_prev = !!(LL_GPIO_ReadInputPort(USER_SW2_GPIO_Port) & USER_SW1_Pin);
    int32_t sw2_prev = !!(LL_GPIO_ReadInputPort(USER_SW2_GPIO_Port) & USER_SW2_Pin);
    while(1) {
        int32_t sw1 = !!(LL_GPIO_ReadInputPort(USER_SW1_GPIO_Port) & USER_SW1_Pin);
        int32_t sw2 = !!(LL_GPIO_ReadInputPort(USER_SW2_GPIO_Port) & USER_SW2_Pin);
        if (sw2_prev & !sw2) {
            // assume that the user doesn't press the button so fast that an annotation fails to get
            // written to disk before the next one is ready.
            memset(buf, ' ', sizeof(buf));
            strcpy(buf, "annotation number:");
            my_itoa_padded(buf + 20, annotation_number, 10);
            strcpy(buf+32, "timestamp:");
            my_itoa_padded(buf + 32 + 11, uwTick, 10);

            sd_write_request.card_addr = ANNOTATION_SD_OFFSET + annotation_number;
            sd_driver_enqueue_request(&sd_write_request);
            annotation_number++;
        }

        // just crammed this into the 'annotator task' even though it's reset.
        // on button release, reset. TODO: would be good to cut the SD card's power for, say, 50ms.
        if (!sw1_prev & sw1) {
            #if 0
            __disable_irq();
            SCB->AIRCR = (0x5fa << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk;
            while(1);
            #endif
        }

        sw1_prev = sw1;
        sw2_prev = sw2;

        vTaskDelay(10);
    }
}
