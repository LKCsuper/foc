/*
 * @Description: 
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2023-10-04 21:09:10
 * @FilePath: \foc\App\motor\utils.c
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
motor_all_state_t motor1;
motor_all_state_t motor2;
MOTOR_PARAM_S motor;
mc_configuration motorCfg;
//MOTOR_PARAM_S motor;
motor_all_state_t *gpstMotor1 = &motor1;
motor_all_state_t *gpstMotor2 = &motor2;
MOTOR_PARAM_S *gpstMotor = &motor;
mc_configuration *gpstMotorCfg = &motorCfg;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 电机参数初始化
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID MOTOR_Param_Init(VOID)
{
    /* 电阻 电感 磁链 */
    gpstMotorCfg->foc_motor_r = 297.80;                 /* m欧 */
    gpstMotorCfg->foc_motor_l = 376.48;                 /* uh */
    gpstMotorCfg->foc_motor_flux_linkage = 12.13;       /* mWB */

    return;
}
#ifdef __cplusplus
}
#endif
