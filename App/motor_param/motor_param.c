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

    /* 开环 V/F 默认参数 — 24V 母线 / 7 对极 / 额定 4000RPM */
    gpstMotorDefault->ol_enable      = true;
    gpstMotorDefault->ol_mode        = 1;       /* 0=V/F, 1=角度开环+电流闭环 */
    gpstMotorDefault->ol_target_rpm  = 80.0f;    /* 目标转速 RPM — 改这一个 */
    gpstMotorDefault->ol_max_rpm     = 800.0f;   /* 开环目标转速上限 */
    gpstMotorDefault->ol_v_boost     = 3.0f;    /* 启动补偿 V */
    gpstMotorDefault->ol_kv_f        = 0.15f;   /* V/Hz */
    gpstMotorDefault->ol_ramp_rpm_s  = 10.0f;   /* 软启动速率 — 越慢越稳 */
    gpstMotorDefault->ol_vq_limit    = 18.0f;   /* 最大 75% 调制 */
    gpstMotorDefault->ol_foc_freq    = 30000.0f; /* FOC 控制频率 Hz */
    gpstMotorDefault->ol_pole_pairs  = 4.0f;     /* 电机极对数 */
    gpstMotorDefault->ol_min_rpm     = 30.0f;    /* 最低转速地板 RPM */
    /* 开环电流模式参数 */
    gpstMotorDefault->ol_iq_set      = 1.0f;     /* 目标 Iq 电流 A */
    gpstMotorDefault->ol_id_set      = 0.0f;     /* 目标 Id 电流 A */
    gpstMotorDefault->ol_current_kp  = 0.5f;     /* 电流环 Kp */
    gpstMotorDefault->ol_current_ki  = 100.0f;   /* 电流环 Ki */
    gpstMotorDefault->ol_start_phase_boost = M_PI / 2.0f; /* 启动 90° 相位补偿 */

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

    // TODO：调试使用
    /* 开发阶段：开环参数始终从代码默认值覆盖，改 motor_param.c 即可生效 */
    gpstMotor->ol_enable      = gpstMotorDefault->ol_enable;
    gpstMotor->ol_mode        = gpstMotorDefault->ol_mode;
    gpstMotor->ol_target_rpm  = gpstMotorDefault->ol_target_rpm;
    gpstMotor->ol_max_rpm     = gpstMotorDefault->ol_max_rpm;
    gpstMotor->ol_v_boost     = gpstMotorDefault->ol_v_boost;
    gpstMotor->ol_kv_f        = gpstMotorDefault->ol_kv_f;
    gpstMotor->ol_ramp_rpm_s  = gpstMotorDefault->ol_ramp_rpm_s;
    gpstMotor->ol_vq_limit    = gpstMotorDefault->ol_vq_limit;
    gpstMotor->ol_foc_freq    = gpstMotorDefault->ol_foc_freq;
    gpstMotor->ol_pole_pairs  = gpstMotorDefault->ol_pole_pairs;
    gpstMotor->ol_min_rpm     = gpstMotorDefault->ol_min_rpm;
    /* 开环电流模式参数 */
    gpstMotor->ol_iq_set      = gpstMotorDefault->ol_iq_set;
    gpstMotor->ol_id_set      = gpstMotorDefault->ol_id_set;
    gpstMotor->ol_current_kp  = gpstMotorDefault->ol_current_kp;
    gpstMotor->ol_current_ki  = gpstMotorDefault->ol_current_ki;
    gpstMotor->ol_start_phase_boost = gpstMotorDefault->ol_start_phase_boost;

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
