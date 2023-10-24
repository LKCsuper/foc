#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 检测
 * @detail: 
 * @param {ULONG} ulDetectType
 * @return {*}
 * @author: lkc
 */
VOID Cmd_Show(void)
{
    PRINTF("Hall [%x]\r\n", utils_read_hall(MOTOR1, 1));

    PRINTF("Hall0 [%d]\r\n", GPIO_ReadInputDataBit(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1));
    PRINTF("Hall1 [%d]\r\n", GPIO_ReadInputDataBit(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2));
    PRINTF("Hall2 [%d]\r\n", GPIO_ReadInputDataBit(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3));

    return;
}

SHELL_EXPORT_CMD(
SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC),
show, 
Cmd_Show, 
show param
);
#ifdef __cplusplus
}
#endif