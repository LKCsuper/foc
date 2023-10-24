/*
 * @Description: 
 * @Date: 2023-02-17 22:55:09
 * @LastEditTime: 2023-10-09 20:34:09
 * @FilePath: \my_foc\App\User\includes.h
 */
#ifndef _FREERTOS_PORT_H
#define _FREERTOS_PORT_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* 不同任务的优先级 */
#define PRI_CMD             ((osPriority_t)(osPriorityRealtime - 1))
#define PRI_INIT            ((osPriority_t)(osPriorityRealtime - 1))

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif

