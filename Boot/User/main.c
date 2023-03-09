/*
 * @Description: 
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2022-11-28 19:28:49
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2023-01-14 18:15:03
 */
#ifdef __cplusplus
extern "C"
{
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

int main(void)
{
    /* 判断是否tftp升级标志位,如果存在直接先打印升级,然后清空标志位 */

    static unsigned char buf[1024];
    int ret = 0;

    /* 初始化ymodem外设 */
    Ymodem_Uart_Init(115200);
    /* 看门狗初始化，防止跑飞 */
    Bsp_Misc_Iwdg(IWDG_Prescaler_256, 0x0FFF);

    FLASH_DisableWriteProtection();
    /* ymodem升级,建议先升级到备份区 */
    ret = Ymodem_Receive(buf);

    if (ret > 0)
    {
        Ymodem_ShowFileInfo();
        /* 代表ymodem升级完成 写入flash标志 */
        Ymodem_Flash_Set_Flag(true);
    }
    else if (ret == ERROR_NO_ACK)
    {
        Y_DEBUG("\r\nYMODEM: Ymodem is not in use \r\n");
    }

    /* 判断是否存在flash升级标志,包括tftp升级或者ymodem */
    if (YMODEM_FLAG == Ymodem_Flash_Get_Flag())
    {
        /* 拷贝备份区到app */
        FLASH_CopyApp();

        /* 清除ymodem升级标志 */
        Ymodem_Flash_Set_Flag(false);
    }

    FLASH_EnableWriteProtection();
    /* 跳转app */
    Ymodem_Jump_App();

    while (1)
    {
        
    }
}

#ifdef __cplusplus
}
#endif
