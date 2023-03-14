/*
 * @Description: 
 * @Date: 2023-02-17 22:55:09
 * @LastEditTime: 2023-03-14 22:52:23
 * @FilePath: \my_foc\App\User\includes.h
 */
#ifndef _INCLUDES_H
#define _INCLUDES_H
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

#include "arm_math.h"
#include "stm32f4xx.h"

/* freertos */
#include "FreeRTOS.h"
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

/* commn */
#include "printf_color.h"
#include "common.h"

/* common */
#include "motor_utils.h"

/* bsp */
#include "bsp_adc.h"
#include "bsp_gpio.h"
#include "bsp_usart.h"
#include "bsp_misc.h"
#include "bsp_tim.h"
#include "bsp_dma.h"
#include "bsp_flash.h"
#include "bsp.h"

/* it */
#include "stm32f4xx_it.h"

/* motor */
#include "motor_common.h"
#include "datatypes.h"
#include "hw_hd75.h"
#include "hw.h"
#include "motor_param.h"
#include "motor_task.h"
#include "motor_foc.h"
#include "motor_it.h"
#include "motor_pid.h"
#include "motor_tim.h"

/* letter shell */
#include "shell.h"
#include "shell_port.h"

/* cmd */
#include "command.h"
#include "cmd_serial_plot.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif

