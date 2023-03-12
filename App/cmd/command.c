/*
 * @Description: 
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2023-03-12 13:52:44
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
/* 任务栈 */
static StackType_t  taskCmdStack[128];
static StaticTask_t taskCmdTcb; //CreateAppTask任务控制块 
const osThreadAttr_t taskCmdAttr = 
{
    .name = "Task Init",
    /* 注意如果超出这个中断最大,任务将创建失败 */
    .priority = PRI_CMD,
    .cb_mem = &taskCmdTcb,
    .cb_size = sizeof(taskCmdTcb),
    .stack_mem = &taskCmdStack,
    .stack_size = sizeof(taskCmdStack),
};

QueueHandle_t queueCmdHandle;
typedef ULONG CMD_QUEUE_ITEM;								//队列项目定义类型
#define CMD_QUEUE_LENGTH	1								//队列项目长度
#define CMD_QUEUE_SIZE		sizeof(CMD_QUEUE_ITEM)		    //队列项目字节数

extern UCHAR gucU3RxBuf[128];
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 获取字符
 * @detail: 
 * @param {uint8_t} *ch
 * @return {*}
 * @author: lkc
 */
int cmd_get_char(uint8_t *ch)
{
    return Bsp_Usart_GetChar(ch);
}

/**
 * @description: 输出字符
 * @detail: 
 * @param {uint8_t} ch
 * @return {*}
 * @author: lkc
 */
int cmd_put_char(uint8_t ch)
{
    Bsp_Usart_PutChar(ch);
    return 1;
}

/**
 * @description: 命令行任务
 * @detail: 
 * @return {*}
 * @author: lkc
 */
void Cmd_Task(void *argument)
{
    PRINTF("Cmd Task Init\r\n");
    shell_init();
    
    STATIC ULONG ulRxDataLen;
    ULONG i = 0;
    /* 创建队列用于接收DMA */
    queueCmdHandle = osMessageQueueNew(CMD_QUEUE_LENGTH, CMD_QUEUE_SIZE, NULL);

    while (1)
    {
        /* 等待队列 */
        osMessageQueueGet(queueCmdHandle, &ulRxDataLen, NULL, portMAX_DELAY);

        for (i = 0; i < ulRxDataLen; i++)
        {
            shell(gucU3RxBuf[i]);
        }
    }
}

/**
 * @description: 命令调试初始化
 * @detail: 
 * @return {*}
 * @author: lkc
 */
void Command_Init(void)
{
    osThreadNew(Cmd_Task, NULL, &taskCmdAttr);
    
    return;
}

#ifdef __cplusplus
}
#endif
