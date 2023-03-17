/*
 * @Description: 
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2022-11-19 09:57:21
 * @LastEditors: lkc
 * @LastEditTime: 2023-03-16 22:35:37
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
 * @description: 是否超时函数
 * @detail: 
 * @return {*}
 * @author: lkc
 */
ULONG Common_Time_Out(VOID)
{
    STATIC ULONG ulTim = 6000;

    if (0 == ulTim)
    {
        return TRUE;
    }

    ulTim--;
    return FALSE
}

#ifdef __cplusplus
}
#endif
