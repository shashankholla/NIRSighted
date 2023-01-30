/**
 * This task initializes the hm01b0 camera, the iCE40 FPGA, and then reads camera frames from the
 * iCE40.
 */

#include "main.h"
#include "cmsis_os.h"

/**
 * Handle to allow EXTI0_IRQHandler to wake up this task.
 */
volatile TaskHandle_t camera_read_task_handle = { 0 };

/**
 * A bunch of statically allocated buffers for storing camera images.
 */
static uint8_t camera_buffers [16][8192] __attribute__((aligned(4)));
int active_camera_buffer = 0;

void camera_read_task(void* param)
{
    camera_read_task_handle = xTaskGetCurrentTaskHandle();

    // USART1 should only interrupt on RX not empty.
    //NVIC_SetPriority(USART1_IRQn, 0);   // TODO: get correct max priority for FreeRTOS
    //NVIC_EnableIRQ(USART1_IRQn);
    //USART1->CR1 |= (1 << 5);

    // enable dma for reception
    USART1->CR3 |= (1 << 6);

    //while(1);

    // TODO: initialize HM01B0

    active_camera_buffer = 0;

    // DMAMUX channel 2 is connected to DMA1 channel 2
    DMAMUX1_Channel2->CCR = DMA_REQUEST_USART1_RX;

    // setup a new USART1 dma xfer
    // use DMA1 channel 2.
    DMA1_Channel2->CPAR  = &USART1->RDR;
    DMA1_Channel2->CMAR  = &camera_buffers[active_camera_buffer][0];
    DMA1_Channel2->CNDTR = 0xc000;
    DMA1_Channel2->CCR = ((0b0  << DMA_CCR_MEM2MEM_Pos) |    // mem2mem mode disabled
                          (0b01 << DMA_CCR_PL_Pos) |         // priority level: medium
                          (0b00 << DMA_CCR_MSIZE_Pos) |      // memory xfer size: 8 bits
                          (0b00 << DMA_CCR_PSIZE_Pos) |      // peripheral xfer size: 8 bits
                          (0b1  << DMA_CCR_MINC_Pos)  |      // memory increment mode: enabled
                          (0b0  << DMA_CCR_PINC_Pos)  |      // periph increment mode: disabled
                          (0b0  << DMA_CCR_CIRC_Pos)  |      // circular mode: disabled
                          (0b0  << DMA_CCR_DIR_Pos)   |      // data transfer dir: read from periph
                          (0b0  << DMA_CCR_TEIE_Pos)  |      // xfer error int en: disabled
                          (0b0  << DMA_CCR_HTIE_Pos)  |      // half xfer int en: disabled
                          (0b0  << DMA_CCR_TCIE_Pos)  |      // xfer complete int en: disabled
                          (0b1  << DMA_CCR_EN_Pos));         // enable channel

    active_camera_buffer = (active_camera_buffer == 15) ? (0) : (active_camera_buffer + 1);

    while (1) {
        // sleep until exti0 goes off, signalling the end of a frame
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Stop dma xfer
        DMA1_Channel2->CCR = 0;

        // setup a new USART1 dma xfer
        DMA1_Channel2->CPAR  = &USART1->RDR;
        DMA1_Channel2->CMAR  = &camera_buffers[active_camera_buffer][0];
        DMA1_Channel2->CNDTR = 0xc000;
        DMA1_Channel2->CCR = ((0b0  << DMA_CCR_MEM2MEM_Pos) |    // mem2mem mode disabled
                              (0b01 << DMA_CCR_PL_Pos) |         // priority level: medium
                              (0b00 << DMA_CCR_MSIZE_Pos) |      // memory xfer size: 8 bits
                              (0b00 << DMA_CCR_PSIZE_Pos) |      // peripheral xfer size: 8 bits
                              (0b1  << DMA_CCR_MINC_Pos)  |      // memory increment mode: enabled
                              (0b0  << DMA_CCR_PINC_Pos)  |      // periph increment mode: disabled
                              (0b0  << DMA_CCR_CIRC_Pos)  |      // circular mode: disabled
                              (0b0  << DMA_CCR_DIR_Pos)   |      // data transfer dir: read from periph
                              (0b0  << DMA_CCR_TEIE_Pos)  |      // xfer error int en: disabled
                              (0b0  << DMA_CCR_HTIE_Pos)  |      // half xfer int en: disabled
                              (0b0  << DMA_CCR_TCIE_Pos)  |      // xfer complete int en: disabled
                              (0b1  << DMA_CCR_EN_Pos));         // enable channel
        active_camera_buffer = (active_camera_buffer == 15) ? (0) : (active_camera_buffer + 1);

        // TODO: write to SD card

    }
}
