/*
 * @Description: 
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2022-11-19 09:57:21
 * @LastEditors: lkc
 * @LastEditTime: 2023-03-19 20:17:20
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
__WEAK ULONG Common_Time_Out(VOID)
{
    STATIC ULONG ulTim = 6000;

    if (0 == ulTim)
    {
        /* 提示错误 */
        return TRUE;
    }

    ulTim--;
    return FALSE
}

#ifdef __cplusplus
}
#endif
