/*
 * @Description: 
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2023-10-06 21:27:12
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
MOTOR_PARAM_S motorDefault;
mc_configuration motorCfg;
//MOTOR_PARAM_S motor;
motor_all_state_t *gpstMotor1 = &motor1;
motor_all_state_t *gpstMotor2 = &motor2;
MOTOR_PARAM_S *gpstMotor = &motor;
MOTOR_PARAM_S *gpstMotorDefault = &motorDefault;
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
    easyflash_init();
    /* 电阻 电感 磁链 */
    gpstMotorCfg->foc_motor_r = 297.80;                 /* m欧 */
    gpstMotorCfg->foc_motor_l = 376.48;                 /* uh */
    gpstMotorCfg->foc_motor_flux_linkage = 12.13;       /* mWB */
    gpstMotorDefault->e_iq_set = 2.0f;

    size_t saved_value_len;

    /* 如果保存的环境变量大小不一致 */
    if (sizeof(MOTOR_PARAM_S) != ef_get_env_blob("flash_param", gpstMotor, sizeof(MOTOR_PARAM_S), &saved_value_len))
    {
        EfErrCode err = ef_set_env_blob("flash_param", gpstMotorDefault, sizeof(MOTOR_PARAM_S));

        if (EF_NO_ERR != err)
        {
            ERROR("Write Flash Param Err[%d] Size[%d]\r\n", err, sizeof(MOTOR_PARAM_S));
        }

        ef_save_env();
        PRINTF("Default Flash Param\r\n");
    }

    PRINTF("Motor iq set [%f]\r\n", gpstMotor->e_iq_set);

    return;
}
#ifdef __cplusplus
}
#endif
