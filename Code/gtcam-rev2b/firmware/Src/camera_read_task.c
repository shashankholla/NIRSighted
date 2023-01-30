/**
 * This task initializes the hm01b0 camera, the iCE40 FPGA, and then reads camera frames from the
 * iCE40.
 */

#include "main.h"
#include "sd_driver_task.h"
#include "cmsis_os.h"

/**
 * Handle to allow EXTI0_IRQHandler to wake up this task.
 */
volatile TaskHandle_t camera_read_task_handle = { 0 };

/**
 * EXTI0_IRQHandler uses this array to log timestamps for vsync rising edges.
 */
volatile uint32_t vsync_rising_edge_timestamps[2];

// read time in milliseconds of most recent frame.
uint32_t camera_read_task_most_recent_frame_timestamp = 0;

// collect some info about the sd driver so we can log it per frame
extern QueueHandle_t sd_request_queue;
extern uint32_t sd_queue_high_watermark;

#define BLOCKS_PER_ENTRY (32)
#define MAX_FRAME_LENGTH ((BLOCKS_PER_ENTRY - 1) * 512)

#define NUM_CAMERA_BUFFERS 16

/**
 * A bunch of statically allocated buffers for storing camera images.
 */
static uint8_t camera_buffers [NUM_CAMERA_BUFFERS][MAX_FRAME_LENGTH] __attribute__((aligned(4)));
static uint8_t metadata_bufs  [NUM_CAMERA_BUFFERS][512];

static void my_itoa(char* s, uint32_t n);
static void my_itoa_padded(char* s, uint32_t n, int len);
static void hm01b0_init();

void camera_read_task(void* param)
{
    // TODO: initialize camera over i2c2 bus (i2c is on pin PF0/E3 and E2/PF1)
    vTaskDelay(10);

    camera_read_task_handle = xTaskGetCurrentTaskHandle();

    // enable dma for reception
    USART1->CR1 |= (1 << 5);
    USART1->CR3 |= (1 << 6);

    // TODO: initialize HM01B0
    uint32_t frame_count = 0;

    // NB: First 2 frames will be trash data because jpeg pipeline is starting up.

    // setup a new USART1 dma xfer
    // use DMA1 channel 2.
    DMA1_Channel2->CPAR  = &USART1->RDR;
    DMA1_Channel2->CMAR  = &camera_buffers[0][0];
    DMA1_Channel2->CNDTR = MAX_FRAME_LENGTH;
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
                          (0b0  << DMA_CCR_TCIE_Pos));       // xfer complete int en: disabled

    // DMAMUX channel 1 is connected to DMA1 channel 2
    DMAMUX1_Channel1->CCR = DMA_REQUEST_USART1_RX;
    DMAMUX1_ChannelStatus->CFR = (1 << 2);

    DMA1_Channel2->CCR |= (0b1  << DMA_CCR_EN_Pos);         // enable channel

    // release FPGA from reset
    LL_GPIO_SetOutputPin(OV2640_RESETB_GPIO_Port, OV2640_RESETB_Pin);

    sd_driver_rw_request_t sd_metadata_write_request;
    sd_driver_rw_request_t sd_frame_write_request;

    while (1) {
        // sleep until exti0 goes off, signalling the end of a frame on the UART
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Stop dma xfer
        DMA1_Channel2->CCR = 0;

        volatile const uint32_t remaining_bytes = DMA1_Channel2->CNDTR;

        // If there was an overflow on the previous packet, reset the USART. If we don't do this,
        // subsequent packets will all be recorded to have a length of 1 because of a bit of a
        // weird state mismatch w the receiver.
        const uint32_t USART1_ISR = USART1->ISR;
        if (USART1->ISR & ((1 << 3) | (1 << 2) | (1 << 1) | (1 << 0))) {
            USART1->CR1 &= ~(1ul << 0);
            while (USART1->CR1 & (1ul << 0));
            USART1->CR1 |= (1ul << 0);
        }

        // setup a new USART1 dma xfer
        const uint32_t camera_buffer = (frame_count + 1) % NUM_CAMERA_BUFFERS;
        DMA1_Channel2->CPAR  = &USART1->RDR;
        DMA1_Channel2->CMAR  = &camera_buffers[camera_buffer][0];
        DMA1_Channel2->CNDTR = MAX_FRAME_LENGTH;
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

        // Format metadata
        const uint32_t timestamp_this_frame = vsync_rising_edge_timestamps[0];
        const uint32_t prev_camera_buffer = (frame_count) % NUM_CAMERA_BUFFERS;
        const uint32_t bytes_this_frame = (MAX_FRAME_LENGTH - remaining_bytes);
        memset(&metadata_bufs[prev_camera_buffer][0], ' ', sizeof(metadata_bufs[0]));
        strcpy((char*)&metadata_bufs[prev_camera_buffer][0], "frame number:");
        my_itoa_padded((char*)&metadata_bufs[prev_camera_buffer][0 + 14], frame_count, 10);
        strcpy((char*)&metadata_bufs[prev_camera_buffer][32], "bytes:");
        my_itoa_padded((char*)&metadata_bufs[prev_camera_buffer][32 + 7], bytes_this_frame, 5);
        strcpy((char*)&metadata_bufs[prev_camera_buffer][64], "millisecond time:");
        my_itoa_padded((char*)&metadata_bufs[prev_camera_buffer][64 + 18], timestamp_this_frame, 8);
        strcpy((char*)&metadata_bufs[prev_camera_buffer][96], "gtcam build date and time: " __DATE__ " " __TIME__);
        my_itoa_padded((char*)&metadata_bufs[prev_camera_buffer][160 + 13], uxQueueMessagesWaiting(sd_request_queue), 4);
        strcpy((char*)&metadata_bufs[prev_camera_buffer][160], "sd queuelen:");
        my_itoa_padded((char*)&metadata_bufs[prev_camera_buffer][192 + 14], sd_queue_high_watermark, 4);
        strcpy((char*)&metadata_bufs[prev_camera_buffer][192], "sd queue max:");

        // debug data, save status registers in case of crash
        *((uint32_t*)(&metadata_bufs[prev_camera_buffer][224])) = USART1_ISR;

        // Write metadata to SD card
        sd_metadata_write_request.direction = SD_DRIVER_DIRECTION_WRITE;
        sd_metadata_write_request.nblocks = 1;
        sd_metadata_write_request.card_addr = (frame_count * BLOCKS_PER_ENTRY);
        sd_metadata_write_request.buf = &metadata_bufs[prev_camera_buffer][0];
        sd_driver_enqueue_request(&sd_metadata_write_request);

        // Write frame to SD card; frames come once every
        sd_frame_write_request.direction = SD_DRIVER_DIRECTION_WRITE;
        sd_frame_write_request.nblocks = ((bytes_this_frame / 512) + 1);
        sd_frame_write_request.card_addr = ((frame_count * BLOCKS_PER_ENTRY) + 1);
        sd_frame_write_request.buf = &camera_buffers[prev_camera_buffer][0];
        sd_driver_enqueue_request(&sd_frame_write_request);

        // Let the health monitor know that our task is still alive.
        if (bytes_this_frame > 0) {
            camera_read_task_most_recent_frame_timestamp = uwTick;
        }

        // Advance
        frame_count++;
    }
}


static void my_itoa(char* s, uint32_t n)
{
    int i = 0;
    do {
        s[i] = n % 10 + '0';
        n /= 10;
    } while (n);

    for (int j = 0; j < (i >> 1); j++) {
        char tmp = s[j];
        s[j] = s[i - j];
        s[i - j] = tmp;
    }
}


static void my_itoa_padded(char* s, uint32_t n, int len)
{
    for (int i = (len - 1); i >= 0; i--) {
        s[i] = n % 10 + '0';
        n /= 10;
    }
}


void hm01b0_init()
{

}
