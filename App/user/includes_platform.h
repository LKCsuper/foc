/*
 * @Description: 平台层头文件（芯片 / RTOS / BSP / HAL）
 * @Author: lkc
 */
#ifndef _INCLUDES_PLATFORM_H
#define _INCLUDES_PLATFORM_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* c std */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

/* stm */
#ifndef USE_STDPERIPH_DRIVER
#define USE_STDPERIPH_DRIVER    1
#endif
#define HSE_VALUE               ((uint32_t)8000000) /* 外部晶振频率 */
#define HSE_USE                 (1)                 /* 1:外部晶振 0:内部 */

#include "arm_math.h"
#include "stm32f4xx.h"

/* freertos */
#include "FreeRTOS.h"
#include "portable.h"
#include "task.h"
#include "semphr.h"
#include "croutine.h"
#include "event_groups.h"
#include "list.h"
#include "queue.h"
#include "stream_buffer.h"
#include "timers.h"
#include "mpu_prototypes.h"
#include "cmsis_os2.h"
#include "freertos_port.h"

/* common */
#include "printf_color.h"
#include "common.h"

/* jlink */
#include "SEGGER_RTT.h"
#include "SEGGER_RTT_DEBUG.h"

/* fal */
#include <fal.h>

/* flashdb */
#include "flashdb.h"

/* motor utils（板级宏依赖前的通用工具） */
#include "motor_utils.h"

/* hardware board macros */
#include "hw_hd75.h"
#include "hw.h"

/* bsp */
#include "bsp_adc.h"
#include "bsp_gpio.h"
#include "bsp_usart.h"
#include "bsp_misc.h"
#include "bsp_tim.h"
#include "bsp_dma.h"
#include "bsp_flash.h"
#include "bsp_it.h"
#include "bsp_hall.h"
#include "bsp_i2c.h"
#include "bsp_mt6701_ssi.h"
#include "bsp.h"

/* hal — App 只调 HAL，寄存器仅留在 Library/Hal/port */
#include "hal_motor.h"
#include "hal_sensor.h"

/* chip interrupt declarations */
#include "stm32f4xx_it.h"

#ifdef __cplusplus
}
#endif
#endif
