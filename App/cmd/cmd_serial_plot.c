/*
 * @Description: 
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2023-03-04 19:32:47
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
__attribute__((section(".bss.os.thread.cb")))
osRtxThread_t worker_thread_tcb_1;

const osThreadAttr_t gstPlotTask = 
{
    .name = "Serial Plot";
    /* 两种模式,其中join模式是多个线程不断添加,不同的参数 */
    .attr_bits = osThreadDetached;
    .cb_mem = 

};
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 串口任务初始化
 * @detail: 
 * @return {*}
 * @author: lkc
 */
void Serial_TaskInit(void *argument)
{
    while (1)
    {
        // 绘图
        TEXT_PLOT(current, "%f, %f, %f", motor.ia, motor.ib, motor.ic);
    }

    return;
}


void Cmd_Serial_Init(void)
{


    return;
}

#ifdef __cplusplus
}
#endif
