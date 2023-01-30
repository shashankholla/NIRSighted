/**
 *
 */

#include "main.h"
#include "sd_driver_task.h"
#include "cmsis_os.h"

#include "MLX90640_I2C_Driver.h"
#include "MLX90640_API.h"

// publicly visible int that holds the time that the most recent MLX frame was read at.
// would maybe be better for this variable to be allocated in main() and passed in as a pointer.
uint32_t mlx_read_task_most_recent_frame_timestamp = 0;

/**
 * Handle to allow interrupts to notify this task
 */
volatile TaskHandle_t mlx_read_task_handle = { 0 };

/**
 * A bunch of statically allocated buffers for storing MLX images.
 */

#define MLX_FRAME_SIZE (834 * 2)
#define BLOCKS_PER_ENTRY (5)

#define NUMBUFS 16

#define  Rate4HZ      0x03
#define  Rate8HZ      0x04
#define  Rate16HZ     0x05
#define  Rate32HZ     0x06
#define  Rate64HZ     0x07
#define	 RefreshRate  Rate16HZ

static uint8_t mlx_buffers[NUMBUFS][MLX_FRAME_SIZE] __attribute__((aligned(4)));
static uint8_t metadata_bufs[NUMBUFS][512] __attribute__((aligned(4)));

//#define MLX_SD_OFFSET (0x03000000ul)
#define MLX_SD_OFFSET (0x00c00000ul)

static void my_itoa_padded(char* s, uint32_t n, int len)
{
    for (int i = (len - 1); i >= 0; i--) {
        s[i] = n % 10 + '0';
        n /= 10;
    }
}

void mlx_read_task(void* param)
{
    // MLX has a powerdown pin on the ice40 vision board!
    LL_GPIO_SetOutputPin(OV2640_PWDN_GPIO_Port, OV2640_PWDN_Pin);
    vTaskDelay(200);

    // I2C1 should use DMA for RX and TX
    SET_BIT(I2C1->CR1, I2C_CR1_RXDMAEN | I2C_CR1_TXDMAEN);

    // enable FM+ for i2c
    SET_BIT(SYSCFG->CFGR1, SYSCFG_CFGR1_I2C1_FMP);

    // I2C1 interrupts when a reload is required or a transfer is done. A transfer is 'done' when...
    //   * A NACK happens (I2C_ISR NACKF)
    //   * 255 bytes have been transfered and a "transfer reload" is required (I2C_ISR TCR)
    //   * The transfer is complete and a restart or stop condition needs to be generated
    //     by software (I2C_ISR TC)
    //   * The transfer is complete and a stop condition was generated automatically by the hardware
    //     because the autostop bit was set (I2C_ISR STOPF)
    SET_BIT(I2C1->CR1, I2C_CR1_NACKIE | I2C_CR1_STOPIE | I2C_CR1_TCIE);
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 8, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);

    // Initialize DMA1. Use channels 3 and 4 for I2C TX and RX respectively. DMA1 channels 3 and 4
    // are connected to DMAMUX channels 2 and 3 respectively.
    DMAMUX1_Channel2->CCR = DMA_REQUEST_I2C1_TX;
    DMAMUX1_Channel3->CCR = DMA_REQUEST_I2C1_RX;

    uint16_t readbuf[16];

    // Initialize MLX
    MLX90640_SetRefreshRate(0x33, RefreshRate);
    MLX90640_SetChessMode(0x33);

    // save eeprom to first 5 blocks of MLX section
    sd_driver_rw_request_t sd_metadata_write_request;
    sd_driver_rw_request_t sd_frame_write_request;
    MLX90640_DumpEE(0x33, mlx_buffers[0]);

    memset(&metadata_bufs[0][0], ' ', sizeof(metadata_bufs[0]));
    strcpy((char*)&metadata_bufs[0][0], "MLX eeprom");

    sd_metadata_write_request.direction = SD_DRIVER_DIRECTION_WRITE;
    sd_metadata_write_request.nblocks = 1;
    sd_metadata_write_request.card_addr = MLX_SD_OFFSET;
    sd_metadata_write_request.buf = &metadata_bufs[0][0];
    sd_driver_enqueue_request(&sd_metadata_write_request);

    sd_frame_write_request.direction = SD_DRIVER_DIRECTION_WRITE;
    sd_frame_write_request.nblocks = 4;
    sd_frame_write_request.card_addr = 1 + MLX_SD_OFFSET;
    sd_frame_write_request.buf = mlx_buffers[0];
    sd_driver_enqueue_request(&sd_frame_write_request);

    uint32_t frame_count = 1;

    while(1) {
        vTaskDelay(1);

        // poll until new frame is ready.
        do {
            uint16_t status_register;
            MLX90640_I2CRead(0x33, 0x8000, 1, &status_register);

            const uint16_t data_ready = status_register & (1 << 3);
            if (data_ready) {
                break;
            }
            vTaskDelay(1);
        } while (1);
        const uint32_t frame_ready_time = uwTick;
        mlx_read_task_most_recent_frame_timestamp = frame_ready_time;

        // get a new frame
        const uint32_t active_buffer = (frame_count) % NUMBUFS;
        memset(mlx_buffers[active_buffer], 0, sizeof(mlx_buffers[0]));
        int status = MLX90640_GetFrameData(0x33, &mlx_buffers[active_buffer][0]);

        // Format metadata
        const uint32_t bytes_this_frame = 834 * 2;
        memset(&metadata_bufs[active_buffer][0], ' ', sizeof(metadata_bufs[0]));
        strcpy((char*)&metadata_bufs[active_buffer][0], "frame number:");
        my_itoa_padded((char*)&metadata_bufs[active_buffer][0 + 14], frame_count, 10);
        strcpy((char*)&metadata_bufs[active_buffer][32], "bytes:");
        my_itoa_padded((char*)&metadata_bufs[active_buffer][32 + 7], bytes_this_frame, 5);
        strcpy((char*)&metadata_bufs[active_buffer][64], "millisecond time:");
        my_itoa_padded((char*)&metadata_bufs[active_buffer][64 + 18], frame_ready_time, 8);
        strcpy((char*)&metadata_bufs[active_buffer][92], "gtcam build date and time: " __DATE__ " " __TIME__);

        // Write metadata to SD card
        sd_metadata_write_request.direction = SD_DRIVER_DIRECTION_WRITE;
        sd_metadata_write_request.nblocks = 1;
        sd_metadata_write_request.card_addr = (frame_count * BLOCKS_PER_ENTRY) + MLX_SD_OFFSET;
        sd_metadata_write_request.buf = &metadata_bufs[active_buffer][0];
        sd_driver_enqueue_request(&sd_metadata_write_request);

        // Write frame to SD card
        sd_frame_write_request.direction = SD_DRIVER_DIRECTION_WRITE;
        sd_frame_write_request.nblocks = 4;
        sd_frame_write_request.card_addr = ((frame_count * BLOCKS_PER_ENTRY) + 1) + MLX_SD_OFFSET;
        sd_frame_write_request.buf = &mlx_buffers[active_buffer][0];
        sd_driver_enqueue_request(&sd_frame_write_request);

        // Advance
        frame_count++;
    }
}
