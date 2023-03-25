/*
 * @Description: 
 * @Version: 2.0
 * @Date: 2023-03-21 23:47:18
 * @LastEditors: lkc
 * @detail: 
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
 * @description: 
 * @detail: 
 * @param {void} *pvParameters
 * @return {*}
 * @author: lkc
 */
VOID Cmd_Detect_Offset(VOID)
{
    
    return;
}


VOID Cmd_Detect(ULONG ulDetectType)
{

    return;
}


SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC),
                detect, Cmd_Detect, detect);

#ifdef __cplusplus
}
#endif
