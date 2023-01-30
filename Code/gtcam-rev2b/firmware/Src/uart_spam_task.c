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
    char* str = param;
    while(1) {
        uint8_t idx = 0;
        while(1) {
            volatile USART_TypeDef* uart = UART4;
            if (READ_BIT(UART4->ISR, USART_ISR_TXE_TXFNF)) {
                UART4->TDR = str[idx];
                idx++;
                if (str[idx] == '\0') {
                    idx = 0;
                    taskYIELD();
                }
            }
        }
    }
}
