/**
 * This test task tries to lock a mutex, and spam the UART.
 *
 * Space for this task isn't declared here. Multiple copies of this thread can run, so it should be
 * declared elsewhere.
 */

#include <stdint.h>

#include "main.h"
#include "cmsis_os.h"

void uart_spam_task(void* param)
{
    while(1) {
//        uint8_t idx = 1;
//        UART4->TDR = 'a';

        while(1) {
//            volatile USART_TypeDef* uart = UART4;
//            if (READ_BIT(UART4->ISR, USART_ISR_TXE_TXFNF)) {
//                UART4->TDR = 'a' + idx;
//                idx = (idx == 25) ? 0 : idx + 1;
//            }
        }
        taskYIELD();
    }
}
