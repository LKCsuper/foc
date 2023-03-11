/*
 * @Description: 
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2023-03-11 07:45:17
 * @FilePath: \foc\Library\Bsp\bsp_dma.c
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 串口任务初始化
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Serial_TaskInit(VOID *argument)
{

    while (1)
    {
        // 绘图
        TEXT_PLOT(current, "%f, %f, %f", motor.ia, motor.ib, motor.ic);
    }
}

/**
 * @description: 
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Cmd_Serial_Init(VOID)
{
    return;
}

#ifdef __cplusplus
}
#endif
