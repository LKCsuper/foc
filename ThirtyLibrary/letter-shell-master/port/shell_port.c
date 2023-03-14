/**
 * @file shell_port.c
 * @author Letter (NevermindZZT@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2019-02-22
 * 
 * @copyright (c) 2019 Letter
 * 
 */

#include "includes.h"

Shell shell;
char shellBuffer[512];

static SemaphoreHandle_t shellMutex;

/**
 * @brief 用户shell写
 * 
 * @param data 数据
 * @param len 数据长度
 * 
 * @return short 实际写入的数据长度
 */
short userShellWrite(char *data, unsigned short len)
{
    //serialTransmit(&debugSerial, (uint8_t *)data, len, 0x1FF);
    Bsp_Usart_PutBuf((uint8_t *)data, len);
    return len;
}


/**
 * @brief 用户shell读
 * 
 * @param data 数据
 * @param len 数据长度
 * 
 * @return short 实际读取到
 */
short userShellRead(char *data, unsigned short len)
{
    //return serialReceive(&debugSerial, (uint8_t *)data, len, 0);
    return Bsp_Usart_GetBuf((uint8_t *)data, len);
}

/**
 * @brief 用户shell上锁
 * 
 * @param shell shell
 * 
 * @return int 0
 */
int userShellLock(Shell *shell)
{
    //xSemaphoreTakeRecursive(shellMutex, portMAX_DELAY);
    osMutexAcquire(shellMutex, portMAX_DELAY);
    return 0;
}

/**
 * @brief 用户shell解锁
 * 
 * @param shell shell
 * 
 * @return int 0
 */
int userShellUnlock(Shell *shell)
{
    //xSemaphoreGiveRecursive(shellMutex);
    osMutexRelease(shellMutex);
    return 0;
}

/**
 * @brief 用户shell初始化
 * 
 */
void userShellInit(void)
{
    //shellMutex = xSemaphoreCreateMutex();
    shellMutex = osMutexNew(NULL);

    shell.write = userShellWrite;
    shell.read = userShellRead;
    shell.lock = userShellLock;
    shell.unlock = userShellUnlock;
    shellInit(&shell, shellBuffer, 512);

    /* 这里不采用任务轮询,我们使用串口dma方式 */
    // if (xTaskCreate(shellTask, "shell", 256, &shell, 5, NULL) != pdPASS)
    // {
    //     //logError("shell task creat failed");
    //     PRINTF("创建任务失败\r\n");
    // }
}
CEVENT_EXPORT(EVENT_INIT_STAGE2, userShellInit);

