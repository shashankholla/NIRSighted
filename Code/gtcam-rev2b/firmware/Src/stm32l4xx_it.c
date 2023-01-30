/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_it.h"
#include "FreeRTOS.h"
#include "task.h"

#include "sd_driver_task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/* USER CODE BEGIN 1 */
#define _SDMMC_STA_CMD_FINISHED_BITS()

extern volatile sd_driver_irq_state_t sdmmc1_irq_state;
extern TaskHandle_t sd_driver_task_handle;

//void SDMMC_launch_cmd()


// TODO: seperate out state machine advancement logic; there's a lot of duplicated code!!!
void SDMMC1_IRQHandler()
{
    const uint32_t sta = SDMMC1->STA;
    BaseType_t wake_higher_priority_task = pdFALSE;

    switch(sdmmc1_irq_state.operation_phase) {
        case SD_DRIVER_OPERATION_PHASE_COMMAND: {
            if (sta & (SDMMC_FLAG_CCRCFAIL | SDMMC_FLAG_CTIMEOUT)) {
                // TODO: retry command.
                while(1);
            } else {
                switch (sdmmc1_irq_state.request_state) {
                    case SD_DRIVER_REQUEST_STATE_WRITE_CMD23_CMD55: {
                        // TODO: check the CMD55 response
                        const uint32_t response = SDMMC1->RESP1;

                        // clear status flags
                        SDMMC1->ICR = SDMMC_FLAG_CMDREND;

                        // set up a CMD23 to dictate the number of blocks.
                        SDMMC1->ARG = sdmmc1_irq_state.nblocks;
                        MODIFY_REG(SDMMC1->CMD, CMD_CLEAR_MASK,
                                   ((1 << 12) | // enable CPSM to send command
                                    (0 << 11) | // don't wait on pending data xfer
                                    (0 << 10) | // don't wait on pending SDIO interrupt
                                    (0b01 << 8) | // short response with CRC
                                    (23 << 0)));

                        // change interrupt mask: ACMD23 has a response type of R1, so we expect an
                        // interrupt on CMDREND
                        SDMMC1->MASK = SDMMC_FLAG_CCRCFAIL | SDMMC_FLAG_CTIMEOUT | SDMMC_FLAG_CMDREND;

                        // advance state machine
                        sdmmc1_irq_state.request_state = SD_DRIVER_REQUEST_STATE_WRITE_CMD23;
                        //sdmmc1_irq_state.operation_phase = SD_DRIVER_OPERATION_PHASE_COMMAND;
                        break;
                    }

                    case SD_DRIVER_REQUEST_STATE_WRITE_CMD23: {
                        // TODO: check the CMD23 response
                        const uint32_t response = SDMMC1->RESP1;

                        // clear status flags
                        SDMMC1->ICR = SDMMC_FLAG_CMDREND;

                        // setup DPSM for a multi-block write
                        SDMMC1->DTIMER = 0xfffffffful;
                        SDMMC1->DLEN   = sdmmc1_irq_state.nblocks << 9;
                        SDMMC1->DCTRL  = ((0b1001 << 4) |   // 512B block
                                          (0b11   << 2) |   // transfer mode: block ending with CMD12.
                                          (0b0    << 1) |   // direction: host-to-card
                                          (0b0    << 0));   // wait for CPSM to do xfer

                        SDMMC1->IDMABASE0 = sdmmc1_irq_state.buf;
                        SDMMC1->IDMACTRL = 1;

                        // set up a CMD25 to initiate data transfer.
                        SDMMC1->CMD |= SDMMC_CMD_CMDTRANS;
                        SDMMC1->ARG = sdmmc1_irq_state.addr;
                        MODIFY_REG(SDMMC1->CMD, CMD_CLEAR_MASK,
                                   ((1 << 12) | // enable CPSM to send command
                                    (0 << 11) | // don't wait on pending data xfer
                                    (0 << 10) | // don't wait on pending SDIO interrupt
                                    (0b01 << 8) | // short response with CRC
                                    (25 << 0)));

                        // change interrupt mask: CMD25 has a response type of R1, so we expect an
                        // interrupt on CMDREND
                        SDMMC1->MASK = SDMMC_FLAG_CCRCFAIL | SDMMC_FLAG_CTIMEOUT | SDMMC_FLAG_CMDREND;

                        // advance state machine
                        sdmmc1_irq_state.request_state = SD_DRIVER_REQUEST_STATE_WRITE_CMD25;
                        //sdmmc1_irq_state.operation_phase = SD_DRIVER_OPERATION_PHASE_COMMAND;
                        break;
                    }

                    case SD_DRIVER_REQUEST_STATE_WRITE_CMD25: {
                        // nothing to do here except clear some flags and confirm that the
                        // transaction is getting off to a good start.

                        // TODO: check the CMD25 response
                        const uint32_t response = SDMMC1->RESP1;

                        // clear statuses
                        SDMMC1->ICR = SDMMC_FLAG_CMDREND;

                        // change interrupt mask: we now want to wait until DATAEND.
                        SDMMC1->MASK = SDMMC_FLAG_DATAEND | SDMMC_FLAG_DTIMEOUT | SDMMC_FLAG_DCRCFAIL;

                        // advance state machine
                        sdmmc1_irq_state.request_state = SD_DRIVER_REQUEST_STATE_WRITE_CMD25;
                        sdmmc1_irq_state.operation_phase = SD_DRIVER_OPERATION_PHASE_DATA;
                        break;
                    }

                    case SD_DRIVER_REQUEST_STATE_WRITE_CMD12: {
                        if (sta & SDMMC_FLAG_CMDREND) {
                            // TODO: check the CMD12 response
                            const uint32_t response = SDMMC1->RESP1;

                            // clear status flag
                            SDMMC1->ICR = SDMMC_FLAG_CMDREND;

                            if (!(sta & SDMMC_FLAG_BUSYD0)) {
                                // advance state machine
                                sdmmc1_irq_state.request_state = SD_DRIVER_REQUEST_STATE_IDLE;
                                sdmmc1_irq_state.operation_phase = SD_DRIVER_OPERATION_PHASE_COMMAND;

                                // change interrupt mask; transaction is now over.
                                SDMMC1->MASK = 0;

                                // wakeup driver thread here to begin next xfer.
                                vTaskNotifyGiveFromISR(sd_driver_task_handle,
                                                       &wake_higher_priority_task);
                            }
                        } else if (sta & SDMMC_FLAG_BUSYD0END) {
                            // TODO: check the CMD12 response
                            const uint32_t response = SDMMC1->RESP1;

                            // clear status flag
                            SDMMC1->ICR = SDMMC_FLAG_BUSYD0END;

                            // advance state machine
                            sdmmc1_irq_state.request_state = SD_DRIVER_REQUEST_STATE_IDLE;
                            sdmmc1_irq_state.operation_phase = SD_DRIVER_OPERATION_PHASE_COMMAND;

                            // change interrupt mask; transaction is now over.
                            SDMMC1->MASK = 0;

                            // wakeup driver thread here to begin next xfer.
                            vTaskNotifyGiveFromISR(sd_driver_task_handle,
                                                          &wake_higher_priority_task);
                        } else {
                            // should never get here
                            while(1);
                        }

                        break;
                    }

                    default: {
                        while(1);
                    }
                }
            }

            break;
        }

        case SD_DRIVER_OPERATION_PHASE_DATA: {
            if (sta & (SDMMC_FLAG_DTIMEOUT | SDMMC_FLAG_DCRCFAIL)) {
                // TODO: retry data xfer in case of timeout or CRC fail.
                while(1);
            } else {
                switch (sdmmc1_irq_state.request_state) {
                    case SD_DRIVER_REQUEST_STATE_WRITE_CMD25: {
                        // I don't think that there are any statuses that we need to check.

                        // clear status flags
                        SDMMC1->ICR |= SDMMC_FLAG_DATAEND;

                        // send CMD12
                        SDMMC1->CMD &= ~SDMMC_CMD_CMDTRANS;
                        SDMMC1->CMD |= SDMMC_CMD_CMDSTOP;

                        SDMMC1->ARG = 0;
                        MODIFY_REG(SDMMC1->CMD, CMD_CLEAR_MASK,
                                   ((1 << 12) | // enable CPSM to send command
                                    (0 << 11) | // don't wait on pending data xfer
                                    (0 << 10) | // don't wait on pending SDIO interrupt
                                    (0b01 << 8) | // short response with CRC
                                    (12 << 0)));

                        // change interrupt mask: CMD12 has a response type of R1b.
                        SDMMC1->MASK = SDMMC_FLAG_CCRCFAIL | SDMMC_FLAG_CTIMEOUT | SDMMC_FLAG_CMDREND | SDMMC_FLAG_BUSYD0END;

                        // advance state machine
                        sdmmc1_irq_state.request_state = SD_DRIVER_REQUEST_STATE_WRITE_CMD12;
                        sdmmc1_irq_state.operation_phase = SD_DRIVER_OPERATION_PHASE_COMMAND;
                        break;
                    }

                    default: {
                        while(1);
                    }
                }
                break;
            }
        }
    }

    portYIELD_FROM_ISR(wake_higher_priority_task);
}


/**
 * EXTI0 is connected to PE0, which is "vsync".
 */
extern volatile TaskHandle_t camera_read_task_handle;
extern volatile uint32_t vsync_rising_edge_timestamps[2];
void EXTI0_IRQHandler(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (EXTI->PR1 & (1 << 0)) {
        // read the gpio register to see whether we had a rising or falling edge
        const uint32_t pe = GPIOE->IDR & (1 << 0);

        if (!(pe)) {
            // Whenever we get a falling edge on the VSYNC signal, wakeup the camera_read_task
            // thread. It will have a few milliseconds to setup a new UART rx transfer and start
            // writing the result of the DMA xfer to the SD card.
            vTaskNotifyGiveFromISR(camera_read_task_handle, &higher_priority_task_woken);

            EXTI->PR1 |= (1 << 0);
            //UART4->TDR = 'V';
        } else {
            // we got a rising edge. In this case, we don't need to wake up any threads, we just
            // need to log the time of the rising edge in order to timestamp the camera frames
            // properly.
            vsync_rising_edge_timestamps[0] = vsync_rising_edge_timestamps[1];
            vsync_rising_edge_timestamps[1] = uwTick;

            EXTI->PR1 |= (1 << 0);
        }
    }

    portYIELD_FROM_ISR(&higher_priority_task_woken);
}

void USART1_IRQHandler(void)
{
    const volatile uint32_t sta = USART1->ISR;
    if (sta & (1 << 5)) {
        UART4->TDR = USART1->RDR;
    }

    if (sta & (1 << 3)) {
        USART1->ICR = (1 << 3);
    }
}

/**
 *
 */
extern volatile TaskHandle_t i2c_task_handle;
void I2C1_EV_IRQHandler()
{
    // interrupt on nack or transfer complete.
    // worth noting that there are a few different ways that a transfer can end (autostop, transfer
    // complete, transfer reload needed).
    const volatile uint32_t sta = I2C1->ISR;

    BaseType_t higher_priority_task_woken = pdFALSE;

    i2c_log_event(0x0000);

    if ((sta & I2C_ISR_TC) || (sta & I2C_ISR_TCR)) {
        // If TC or TCR was set, we disable interrupts and pass control back to the task working
        // with the i2c. The task performs the modifications to the i2c needed to clear the status
        // flags and then is responsible for re-enabling interrupts
        CLEAR_BIT(I2C1->CR1, I2C_CR1_NACKIE | I2C_CR1_TCIE);
        vTaskNotifyGiveFromISR(i2c_task_handle, &higher_priority_task_woken);
    }

    if (sta & I2C_ISR_STOPF) {
        SET_BIT(I2C1->ICR, I2C_ICR_STOPCF);
        vTaskNotifyGiveFromISR(i2c_task_handle, &higher_priority_task_woken);
    }

    if (sta & I2C_ISR_NACKF) {
        // gross hack: don't clear the nack flag, let the application code read it... just disable the interrupt.
        // would probably be better to send a value with the give notify.
        CLEAR_BIT(I2C1->CR1, I2C_CR1_NACKIE);
        vTaskNotifyGiveFromISR(i2c_task_handle, &higher_priority_task_woken);
    }

    portYIELD_FROM_ISR(&higher_priority_task_woken);
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
