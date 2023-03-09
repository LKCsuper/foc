/*
 * @Description: 
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2023-03-05 23:13:25
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
    //.cb_mem = 

};

unsigned char ucRunInfo[512];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
//RunTimeStats任务
void Trace_Task(void *pvParameters)
{
	while(1)
	{
        memset(ucRunInfo, 0, sizeof(ucRunInfo));				//信息缓冲区清零
        vTaskGetRunTimeStats(ucRunInfo);		//获取任务运行时间信息
        printf("任务名\t\t\t运行时间\t运行所占百分比\r\n");
        printf("%s\r\n",ucRunInfo);
		
		vTaskDelay(10);                           	//延时10ms，也就是1000个时钟节拍	
	}
}

/**
 * @description: 命令行任务初始化
 * @detail: 
 * @return {*}
 * @author: lkc
 */
void Cmd_Trace_Init(void)
{

    return;
}

#ifdef __cplusplus
}
#endif
