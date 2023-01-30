/**
 * 3/12/2021
 * This file contains a thread which manages sd card insertion, removal, reading, and writing.
 *
 * It might be better for this code to reside entirely in interrupts, but I thought that doing it
 * this way might be easier. Also, it lets us respond to card insertion, which requires blocking
 * code.
 *
 * If we had this code residing entirely in interrupts, we'd save some time by avoiding context
 * switching into this thread whenever data needs to be written.
 *
 * One point of concern: when doing a multi-block write, each DMA xfer will only consist of up to
 * 512 Bytes. Even if the bus is clocked at - say - 1/8th the CPU speed (2 - 8 MHz), the DMA will
 * complete after only about 8000 CPU cycles. This seems a little frequent to me. Because we need
 * to wait until the SD card isn't busy after each block, it won't be entirely straightforward for
 * the DMA to figure out when to begin the next xfer. However... it's likely that the SDMMC block
 * has an "sd card no longer busy" event that we can take advantage of.
 *
 * See SD simplified spec 5.7.2.5 Extension Register Write Command (Multi-Block) for more info.
 *
 * Processes can request that data be written to the buffer cache
 *
 *
 * 3/17/2021
 * Alright, today my aim is to just write some blocks to the SD card over the SDIO bus with blocking
 * DMA xfers.
 *
 * It looks like the STM32CubeMX code is correctly initializing the SD card and leaving it in a
 * state where it's ready to start recieving data. I guess my only concern right now is that the
 * library code has its own interrupt handlers that will interfere somehow with my code, or that
 * it leaves some hardware in an unexpected state. I'd like to know more about the SD card
 * initialization code's side effects.
 */

#include "main.h"
#include "sd_driver_task.h"
#include "cmsis_os.h"

#include <string.h>

////////////////////////////////////////////////////////////////////////////////
// Static variables
volatile sd_driver_irq_state_t sdmmc1_irq_state = { 0 };
extern QueueHandle_t sd_request_queue;

uint32_t sd_driver_task_most_recent_write_timestamp = 0;

uint32_t sd_driver_task_queue_fail_count = 0;

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(SD_HandleTypeDef* hsd)
{
    hsd->Instance = SDMMC1;
    hsd->Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
    hsd->Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    hsd->Init.BusWide = SDMMC_BUS_WIDE_4B;
    hsd->Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd->Init.ClockDiv = 2;
    hsd->Init.Transceiver = SDMMC_TRANSCEIVER_DISABLE;

    // setting TIM1 to specific values here makes HAL_SD_Init fail.
    *((volatile uint32_t*)0x40012c24) = 0x000;
    volatile uint32_t tim1_snapshot = *((volatile uint32_t*)0x40012c24);

    int result = HAL_SD_Init(hsd);

    int tries = 0;
    while ((tries < 8) && (result != HAL_OK)) {
        vTaskDelay(100);
        result = HAL_SD_Init(hsd);
        tries++;
    }

    if (result != HAL_OK) {
        while (1) taskYIELD();
    }

    // set priority of SDMMC interrupt so that it won't pre-empt FreeRTOS critical sections
    HAL_NVIC_SetPriority(SDMMC1_IRQn, 8, 0);
    HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
}

TaskHandle_t sd_driver_task_handle = NULL;

uint32_t sd_queue_high_watermark = 0;
/**
 * Returns nonzero on failure.
 */
int32_t sd_driver_enqueue_request(const sd_driver_rw_request_t* req)
{
    if (sd_request_queue) {
        int32_t retval = xQueueSendToBack(sd_request_queue, (const void*)req, 0);

        const uint32_t queue_size = uxQueueMessagesWaiting(sd_request_queue);
        if (queue_size > sd_queue_high_watermark) {
            sd_queue_high_watermark = queue_size;
        }

        if (retval != pdTRUE) {
            sd_driver_task_queue_fail_count++;
        }
        return retval;
    } else {
        return -1;
    }
}

SD_HandleTypeDef hsd12;

void sd_driver_task(void* arg)
{
    //SD_HandleTypeDef* hsd = (SD_HandleTypeDef*)arg;

    SD_HandleTypeDef* hsd = (SD_HandleTypeDef*)&hsd12;

    volatile SDMMC_TypeDef* sdmmc1 = SDMMC1;

    // make sure that SD card power is turned on.
    LL_GPIO_ResetOutputPin(SD_nENABLE_GPIO_Port, SD_nENABLE_Pin);
    vTaskDelay(1);

    MX_SDMMC1_SD_Init(hsd);

    // this variable is accessed by the IRQ handler so it knows which thread to notify on completion
    // of an SD transfer
    sd_driver_task_handle = xTaskGetCurrentTaskHandle();

    //volatile int stackusage = uxTaskGetStackHighWaterMark(sd_driver_task_handle);

    while(1) {
        // Sleep until there's a request in one of the buffers.
        // TODO: how to handle card insertion and removal as well?? We could just have card
        // insertion and removal as a different type of event in the queue.
        sd_driver_rw_request_t rw_request;
        xQueueReceive(sd_request_queue, &rw_request, portMAX_DELAY);

        // TODO: use buffer cache to allow reads to cut in line

        // start new transfer based on the rw_request that we got
        // unmask SDMMC interrupts for R1 type commands
        SDMMC1->MASK = SDMMC_FLAG_CCRCFAIL | SDMMC_FLAG_CTIMEOUT | SDMMC_FLAG_CMDREND;

        // setup SDMMC interrupt state
        sdmmc1_irq_state.request_state = SD_DRIVER_REQUEST_STATE_WRITE_CMD23_CMD55;
        sdmmc1_irq_state.operation_phase = SD_DRIVER_OPERATION_PHASE_COMMAND;
        sdmmc1_irq_state.nblocks = rw_request.nblocks;
        sdmmc1_irq_state.addr = rw_request.card_addr;
        sdmmc1_irq_state.buf = rw_request.buf;

        // send an appcmd
        SDMMC1->ARG = (uint32_t)(hsd->SdCard.RelCardAdd << 16);
        MODIFY_REG(SDMMC1->CMD, CMD_CLEAR_MASK, (1 << 12) | (0 << 10) | (1 << 8) | (55 << 0));

        // TODO: sleep until transfer is over.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        sd_driver_task_most_recent_write_timestamp = uwTick;
    }
}
