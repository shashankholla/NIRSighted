/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l4xx_ll_adc.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx.h"
#include "stm32l4xx_ll_i2c.h"
#include "stm32l4xx_ll_crs.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_exti.h"
#include "stm32l4xx_ll_cortex.h"
#include "stm32l4xx_ll_utils.h"
#include "stm32l4xx_ll_pwr.h"
#include "stm32l4xx_ll_spi.h"
#include "stm32l4xx_ll_tim.h"
#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l4xx_hal_uart.h"

#include "../saeclib/src/saeclib_circular_buffer.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef struct i2c_sta_log_event {
    uint32_t sta, systick, label;
} i2c_sta_log_event_t;

extern volatile saeclib_circular_buffer_t i2c_sta_log;

    #if 0
#define i2c_log_event(labe) do { \
        saeclib_circular_buffer_disposeone(&i2c_sta_log); \
        __disable_irq(); \
        i2c_sta_log_event_t log = { .sta = I2C1->ISR, .systick = portGET_RUN_TIME_COUNTER_VALUE(), .label = (labe)}; \
        saeclib_circular_buffer_pushone(&i2c_sta_log, &log); \
        __enable_irq(); \
    } while (0)
    #else
#define i2c_log_event(labe)
    #endif


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VUSB_EN_Pin LL_GPIO_PIN_1
#define VUSB_EN_GPIO_Port GPIOD
#define CHG_STAT_Pin LL_GPIO_PIN_8
#define CHG_STAT_GPIO_Port GPIOA
#define OV2640_RESETB_Pin LL_GPIO_PIN_2
#define OV2640_RESETB_GPIO_Port GPIOF
#define OV2640_EXPST_B_Pin LL_GPIO_PIN_13
#define OV2640_EXPST_B_GPIO_Port GPIOC
#define SD_nENABLE_Pin LL_GPIO_PIN_7
#define SD_nENABLE_GPIO_Port GPIOC
#define LSM6DSL_INT2_Pin LL_GPIO_PIN_7
#define LSM6DSL_INT2_GPIO_Port GPIOG
#define OV2640_FREX_Pin LL_GPIO_PIN_4
#define OV2640_FREX_GPIO_Port GPIOF
#define OV2640_PWDN_Pin LL_GPIO_PIN_5
#define OV2640_PWDN_GPIO_Port GPIOF
#define LSM6DSL_INT1_Pin LL_GPIO_PIN_2
#define LSM6DSL_INT1_GPIO_Port GPIOG
#define OV2640_STROBE_Pin LL_GPIO_PIN_10
#define OV2640_STROBE_GPIO_Port GPIOF
#define SD_DETECT_Pin LL_GPIO_PIN_2
#define SD_DETECT_GPIO_Port GPIOC
#define USER_SW2_Pin LL_GPIO_PIN_5
#define USER_SW2_GPIO_Port GPIOA
#define FRAM1_nWP_Pin LL_GPIO_PIN_10
#define FRAM1_nWP_GPIO_Port GPIOE
#define FRAM1_nCS_Pin LL_GPIO_PIN_12
#define FRAM1_nCS_GPIO_Port GPIOB
#define USER_SW1_Pin LL_GPIO_PIN_7
#define USER_SW1_GPIO_Port GPIOA
#define VBUS_DETECT_Pin LL_GPIO_PIN_0
#define VBUS_DETECT_GPIO_Port GPIOB
#define FRAM1_nHOLD_Pin LL_GPIO_PIN_14
#define FRAM1_nHOLD_GPIO_Port GPIOF
#define FRAM2_nCS_Pin LL_GPIO_PIN_1
#define FRAM2_nCS_GPIO_Port GPIOG
#define FRAM2_nHOLD_Pin LL_GPIO_PIN_12
#define FRAM2_nHOLD_GPIO_Port GPIOE
#define FRAM2_nWP_Pin LL_GPIO_PIN_14
#define FRAM2_nWP_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
