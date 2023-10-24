/*
 * @Description: 
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2023-10-10 19:02:14
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
motor_all_state_t m_motor_1;
motor_all_state_t m_motor_2;
mc_configuration mcconf1;
mc_configuration mcconf2;
MOTOR_PARAM_S motor;
MOTOR_PARAM_S motorDefault;
mc_configuration motorCfg;
//MOTOR_PARAM_S motor;
motor_all_state_t *gpstMotor1 = &m_motor_1;
motor_all_state_t *gpstMotor2 = &m_motor_2;
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

    gpstMotor1->m_conf = &mcconf1;
    gpstMotor2->m_conf = &mcconf2;

    motor_all_state_t *motor = gpstMotor1;

    motor->m_conf->m_hall_extra_samples = MCCONF_M_HALL_EXTRA_SAMPLES;

    PRINTF("PARAM %d %d\r\n", motor->m_conf->m_hall_extra_samples, MCCONF_M_HALL_EXTRA_SAMPLES);

    return;
}
#ifdef __cplusplus
}
#endif
