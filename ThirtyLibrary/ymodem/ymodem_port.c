/*
 * @Description: 
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2022-11-19 09:57:21
 * @LastEditors: lkc
 * @LastEditTime: 2023-03-03 22:40:54
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "ymodem_includes.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 串口初始化
 * @detail: 
 * @param {uint32_t} ulBound 波特率
 * @return {*}
 * @author: lkc
 */
void Ymodem_Uart_Init(uint32_t ulBound)
{
    Bsp_Usart_DebugInit(ulBound);
    return;
}

/**
 * @description: 输出到终端一个字符
 * @detail:
 * @param {uint8_t} c 输出字符
 * @return {*}
 * @author: lkc
 */
void Ymodem_PutChar(uint8_t c)
{
    Bsp_Usart_PutChar(c);
    return;
}

/**
 * @description: 获取终端字符
 * @detail:
 * @param {uint8_t} *key
 * @return {*}
 * @author: lkc
 */
uint32_t Ymodem_GetChar(uint8_t *key)
{
    return Bsp_Usart_Get8Bit(key);
}

/**
 * @description: 延时初始化
 * @detail: 
 * @param {unsigned int} ms 毫秒
 * @return {*}
 * @author: lkc
 */
void Ymodem_Delay(unsigned int ms)
{
    osDelay(ms);
    return;
}

/**
 * @description: 升级标志
 * @detail: 主要是ymodem升级到备份区之后然后置位,关于这个升级标志存放位置，可以放在升级app_bak 最后4个字节
 * 这样既不影响程序运行，当擦除升级标志的时候正好一起将app_bak擦掉
 * @param {bool} isSet 是否置位
 * @return {*}
 * @author: lkc
 */
void Ymodem_Flash_Set_Flag(bool isSet)
{
    YMODEM_WARNING("You have to make sure you don't need this function [ %s ] \r\n", __func__);
    return;
}

/**
 * @description: 升级标志
 * @detail: 主要是ymodem升级到备份区之后然后置位
 * @param {bool} isSet 是否置位
 * @return {*}
 * @author: lkc
 */
uint32_t Ymodem_Flash_Get_Flag(void)
{
    YMODEM_WARNING("You have to make sure you don't need this function [ %s ] \r\n", __func__);
    return 0;
}

/**
 * @brief  flash初始化,主要是打开flash写限制
 * @param  None
 * @retval None
 */
void Ymodem_Flash_Init(void)
{
    YMODEM_WARNING("You have to make sure you don't need this function [ %s ] \r\n", __func__);
    return;
}

/**
 * @brief  擦除所有的app部分
 * @param  StartSector: start of user flash area
 * @retval 0: user flash area successfully erased
 *         1: error occurred
 */
void Ymodem_Flash_Erase_App(void)
{
    YMODEM_WARNING("You have to make sure you don't need this function [ %s ] \r\n", __func__);
    return;
}

/**
 * @brief  将数据写入flash中(data are 32-bit aligned).
 * @note   After writing data buffer, the flash content is checked.
 * @param  FlashAddress: start address for writing data buffer
 * @param  Data: pointer on data buffer
 * @param  DataLength: length of data buffer (unit is 32-bit word)
 * @retval 0: Data successfully written to Flash memory
 *         1: Error occurred while writing data in Flash memory
 *         2: Written Data in flash memory is different from expected one
 */
uint32_t Ymodem_Flash_Write(uint32_t *Data, uint32_t DataLength)
{
    YMODEM_WARNING("You have to make sure you don't need this function [ %s ] \r\n", __func__);
    return 0;
}

/**
 * @description: 消息显示
 * @detail:
 * @return {*}
 * @author: lkc
 */
void Ymodem_Info(void)
{
    Y_DEBUG("\r\n========================================================\r\n");
    Y_DEBUG("\r\nTime: [ %s ] [ %s ]\r\n", __DATE__, __TIME__);
    Y_DEBUG("Attention: Do not print during ymodem transfer\r\n");

    return;
}

pFunction Jump_To_Application;
uint32_t JumpAddress;
/**
 * @description: 跳转app
 * @detail: 
 * @return {*}
 * @author: lkc
 */
void Ymodem_Jump_App(void)
{
    Y_DEBUG("YMODEM : Start Jump to App\r\n");
    JumpAddress = *(__IO uint32_t *)(APPLICATION_ADDRESS + 4);
    /* Jump to user application */
    Jump_To_Application = (pFunction)JumpAddress;
    /* Initialize user application's Stack Pointer */
    __set_MSP(*(__IO uint32_t *)APPLICATION_ADDRESS);

    /* 关闭所有已经开启的中断 
        注意看门狗，就算跳转app也不能关闭，除非重新上电
    */
    __disable_irq();
    Jump_To_Application();

    return;
}
#ifdef __cplusplus
}
#endif
