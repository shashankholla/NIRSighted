/**
 * The MLX-provided API functions require the implementation of 2 driver functions:
 *     MLX90640_I2CRead
 *     MLX90640_I2CWrite
 *
 * This file provides implementation of these 2 functions that use DMA and FreeRTOS.
 */

#include "main.h"

#include "MLX90640_I2C_Driver.h"

#include "cmsis_os.h"
#include "stm32l4xx.h"

//#define MLX90640_PERIPHERAL_ADDR 0x33

volatile TaskHandle_t i2c_task_handle;

/**
 * Reads from I2C1. Must be called from a FreeRTOS thread. The thread will sleep until data
 * has been read.
 */
int MLX90640_I2CRead(uint8_t devaddr, uint16_t mlx_readaddr, uint16_t num_addrs_read, uint16_t* data)
{
    i2c_log_event(0x0100);

    // TODO: acquire lock on I2C would be ideal
    i2c_task_handle = xTaskGetCurrentTaskHandle();

    // setup DMA xfer to write device memory address
    uint8_t data_to_write[] = {(mlx_readaddr >> 8) & 0xff,
                               (mlx_readaddr >> 0) & 0xff};
    DMA1_Channel3->CCR = 0;
    DMA1_Channel3->CPAR  = &I2C1->TXDR;
    DMA1_Channel3->CMAR  = data_to_write;
    DMA1_Channel3->CNDTR = 2;
    DMA1_Channel3->CCR = ((0b0  << DMA_CCR_MEM2MEM_Pos) |    // mem2mem mode disabled
                          (0b01 << DMA_CCR_PL_Pos) |         // priority level: medium
                          (0b00 << DMA_CCR_MSIZE_Pos) |      // memory xfer size: 8 bits
                          (0b00 << DMA_CCR_PSIZE_Pos) |      // peripheral xfer size: 8 bits
                          (0b1  << DMA_CCR_MINC_Pos)  |      // memory increment mode: enabled
                          (0b0  << DMA_CCR_PINC_Pos)  |      // periph increment mode: disabled
                          (0b0  << DMA_CCR_CIRC_Pos)  |      // circular mode: disabled
                          (0b1  << DMA_CCR_DIR_Pos)   |      // data transfer dir: write to periph
                          (0b0  << DMA_CCR_TEIE_Pos)  |      // xfer error int en: disabled
                          (0b0  << DMA_CCR_HTIE_Pos)  |      // half xfer int en: disabled
                          (0b0  << DMA_CCR_TCIE_Pos));       // xfer complete int en: disabled

    DMA1_Channel3->CCR |= (0b1  << DMA_CCR_EN_Pos);         // enable channel

    // setup I2C xfer
    SET_BIT(I2C1->ICR, I2C_ICR_STOPCF);
    MODIFY_REG(I2C1->CR2, I2C_CR2_NBYTES_Msk, (2 << I2C_CR2_NBYTES_Pos)); // 2 bytes
    CLEAR_BIT(I2C1->CR2, I2C_CR2_RD_WRN);  // direction: write
    CLEAR_BIT(I2C1->CR2, I2C_CR2_AUTOEND); // no autoend so we can send restart
    MODIFY_REG(I2C1->CR2, I2C_CR2_SADD_Msk, (((uint32_t)devaddr) << 1));

    // start xfer
    SET_BIT(I2C1->CR2, I2C_CR2_START);

    // FreeRTOS take notify; this task gets woken up when there's a NAK or when the xfer is
    // complete
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (I2C1->ISR & I2C_ISR_NACKF) while(1) { vTaskDelay(100); }

    i2c_log_event(0x0101);

    ////////////////////////////////
    // setup DMA xfer to read device
    DMA1_Channel4->CCR = 0;
    DMA1_Channel4->CPAR  = &I2C1->RXDR;
    DMA1_Channel4->CMAR  = ((uint32_t)data);
    DMA1_Channel4->CNDTR = num_addrs_read * 2;
    DMA1_Channel4->CCR = ((0b0  << DMA_CCR_MEM2MEM_Pos) |    // mem2mem mode disabled
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

    DMA1_Channel4->CCR |= (0b1 << DMA_CCR_EN_Pos);

    uint32_t bytes_remaining = num_addrs_read * 2;

    // setup I2C read with stop
    if (bytes_remaining <= 255) {
        // setup transfer less than 255 bytes.
        SET_BIT(I2C1->CR2, I2C_CR2_RD_WRN);
        CLEAR_BIT(I2C1->CR2, I2C_CR2_RELOAD);
        MODIFY_REG(I2C1->CR2, I2C_CR2_SADD_Msk, (((uint32_t)devaddr) << 1));
        MODIFY_REG(I2C1->CR2, I2C_CR2_NBYTES_Msk, (bytes_remaining << I2C_CR2_NBYTES_Pos));

        SET_BIT(I2C1->CR2, I2C_CR2_START);
        asm volatile("nop \n nop \n");
        SET_BIT(I2C1->CR2, I2C_CR2_AUTOEND);

        // re-enable interrupts; the ISR disables the TC interrupt because we need to clear the TC
        // flag ourselves.
        SET_BIT(I2C1->CR1, I2C_CR1_NACKIE | I2C_CR1_TCIE);

        // FreeRTOS take notify; this task gets woken up when there's a NAK or when the xfer is
        // complete
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (I2C1->ISR & I2C_ISR_NACKF) while(1) { vTaskDelay(100); }

        i2c_log_event(0x0102);
    } else {
        SET_BIT(I2C1->CR2, I2C_CR2_RD_WRN);  // direction: read
        SET_BIT(I2C1->CR2, I2C_CR2_RELOAD);
        MODIFY_REG(I2C1->CR2, I2C_CR2_SADD_Msk, (((uint32_t)devaddr) << 1));

        MODIFY_REG(I2C1->CR2, I2C_CR2_NBYTES_Msk, (0xfful << I2C_CR2_NBYTES_Pos));
        SET_BIT(I2C1->CR2, I2C_CR2_RELOAD);
        CLEAR_BIT(I2C1->CR2, I2C_CR2_AUTOEND);
        SET_BIT(I2C1->CR2, I2C_CR2_START);

        while (bytes_remaining > 0) {
            if (bytes_remaining <= 255) {
                MODIFY_REG(I2C1->CR2, I2C_CR2_NBYTES_Msk, (bytes_remaining << I2C_CR2_NBYTES_Pos));
                CLEAR_BIT(I2C1->CR2, I2C_CR2_RELOAD);
                asm volatile("nop \n nop \n");
                SET_BIT(I2C1->CR2, I2C_CR2_AUTOEND);

                bytes_remaining = 0;
            } else {
                MODIFY_REG(I2C1->CR2, I2C_CR2_NBYTES_Msk, (0xfful << I2C_CR2_NBYTES_Pos));
                SET_BIT(I2C1->CR2, I2C_CR2_RELOAD);
                CLEAR_BIT(I2C1->CR2, I2C_CR2_AUTOEND);

                bytes_remaining -= 255;
            }

            // re-enable interrupts; the ISR disables the TC interrupt because we need to clear the TC
            // flag ourselves.
            SET_BIT(I2C1->CR1, I2C_CR1_NACKIE | I2C_CR1_TCIE);

            // FreeRTOS take notify
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            if (I2C1->ISR & I2C_ISR_NACKF) while(1) { vTaskDelay(100); }

            i2c_log_event(0x0103);
        }
    }

    // If we're at the end, but the transaction is still active for some reason, reset the I2C
    if (I2C1->ISR & 0x8000) {
        CLEAR_BIT(I2C1->CR1, I2C_CR1_PE);
        for (volatile int i = 0; i < 2; i++);
        SET_BIT(I2C1->CR1, I2C_CR1_PE);
    }

    // swap endianness of results
    for (int i = 0; i < num_addrs_read; i++) {
        data[i] = __builtin_bswap16(data[i]);
    }

    return 0;
}

/**
 *
 */
int MLX90640_I2CWrite(uint8_t devaddr, uint16_t mlx_writeaddr, uint16_t data)
{
    i2c_log_event(0x0010);

    // TODO: acquire lock on I2C would be ideal
    i2c_task_handle = xTaskGetCurrentTaskHandle();

    // setup DMA xfer: we use DMA1 channel
    uint8_t data_to_write[] = {(mlx_writeaddr >> 8) & 0xff,
                               (mlx_writeaddr >> 0) & 0xff,
                               (data >> 8) & 0xff,
                               (data >> 0) & 0xff};
    // Stop dma xfer
    DMA1_Channel3->CCR = 0;
    DMA1_Channel3->CPAR  = &I2C1->TXDR;
    DMA1_Channel3->CMAR  = data_to_write;
    DMA1_Channel3->CNDTR = 4;
    DMA1_Channel3->CCR = ((0b0  << DMA_CCR_MEM2MEM_Pos) |    // mem2mem mode disabled
                          (0b01 << DMA_CCR_PL_Pos) |         // priority level: medium
                          (0b00 << DMA_CCR_MSIZE_Pos) |      // memory xfer size: 8 bits
                          (0b00 << DMA_CCR_PSIZE_Pos) |      // peripheral xfer size: 8 bits
                          (0b1  << DMA_CCR_MINC_Pos)  |      // memory increment mode: enabled
                          (0b0  << DMA_CCR_PINC_Pos)  |      // periph increment mode: disabled
                          (0b0  << DMA_CCR_CIRC_Pos)  |      // circular mode: disabled
                          (0b1  << DMA_CCR_DIR_Pos)   |      // data transfer dir: write to periph
                          (0b0  << DMA_CCR_TEIE_Pos)  |      // xfer error int en: disabled
                          (0b0  << DMA_CCR_HTIE_Pos)  |      // half xfer int en: disabled
                          (0b0  << DMA_CCR_TCIE_Pos));       // xfer complete int en: disabled

    DMA1_Channel3->CCR |= (0b1  << DMA_CCR_EN_Pos);         // enable channel

    // setup I2C xfer
    // I think that if the STOP flag was set, it will refuse to start unless we clear it.
    //SET_BIT(I2C1->ICR, I2C_ICR_STOPCF);

    CLEAR_BIT(I2C1->CR2, I2C_CR2_RD_WRN);     // direction: write
    SET_BIT(I2C1->CR2, I2C_CR2_AUTOEND);    // enable autoend mode
    CLEAR_BIT(I2C1->CR2, I2C_CR2_RELOAD);
    MODIFY_REG(I2C1->CR2, I2C_CR2_SADD_Msk, (((uint32_t)devaddr) << 1));     // set preipheral addr
    MODIFY_REG(I2C1->CR2, I2C_CR2_NBYTES_Msk, (4 << I2C_CR2_NBYTES_Pos));      // 4 bytes

    // start xfer
    SET_BIT(I2C1->CR2, I2C_CR2_START);

    // FreeRTOS take notify; this task gets woken up when there's a NAK or when the xfer is
    // complete
    //CLEAR_BIT(I2C1->CR1, I2C_CR1_NACKIE | I2C_CR1_TCIE);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (I2C1->ISR & I2C_ISR_NACKF) while(1) { vTaskDelay(100); }

    i2c_log_event(0x0011);

    return 0;
}
