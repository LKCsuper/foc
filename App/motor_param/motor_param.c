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
    gpstMotorCfg->foc_motor_r = 0.4f;                  /* 400mΩ = 0.4Ω */
    gpstMotorCfg->foc_motor_l = 0.0008f;               /* 800uH = 0.0008H */
    gpstMotorCfg->foc_motor_flux_linkage = 0.0091f;    /* 9.1mWb = 0.0091Wb */
    gpstMotorDefault->e_iq_set = 1.5f;

    /* 开环参数 — 当前示例为 24V/2 对极；极对数必须按实际电机修改。 */
    /*
     * 调试阶段上电后保持停机，避免刚烧录新算法就自动冲到高转速。
     * 需要转动时由 Shell 的 olspeed/olstart 明确启动。
     */
    gpstMotorDefault->ol_enable      = false;
    gpstMotorDefault->ol_mode        = MOTOR_OL_MODE_IF;
    gpstMotorDefault->ol_target_rpm  = 500.0f;     /* 首次手动 olstart 的安全目标转速 RPM */
    gpstMotorDefault->ol_max_rpm     = 3000.0f;   /* 开环目标转速上限 */
    gpstMotorDefault->ol_v_boost     = 1.0f;    /* 启动补偿 V */
    gpstMotorDefault->ol_kv_f        = 0.057f;   /* V/Hz */
    gpstMotorDefault->ol_ramp_rpm_s  = 50.0f;    /* 软启动速率 — 越慢越稳 */
    gpstMotorDefault->ol_vq_limit    = 18.0f;   /* 最大 75% 调制 */
    gpstMotorDefault->ol_foc_freq    = 30000.0f; /* ADC-DMA 基准频率；实际 FOC 频率还要除以分频系数 */
    gpstMotorDefault->ol_pole_pairs  = 2.0f;     /* 电机极对数 */
    gpstMotorDefault->ol_min_rpm     = 30.0f;    /* 最低转速地板 RPM */
    /* 开环电流模式参数 */
    gpstMotorDefault->ol_iq_set      = 0.3f;     /* 首次 I/F 起步目标 Iq 电流 A */
    gpstMotorDefault->ol_id_set      = 0.0f;     /* 目标 Id 电流 A */
    gpstMotorDefault->ol_current_kp  = 0.2f;     /* 电流环 Kp */
    gpstMotorDefault->ol_current_ki  = 30.0f;   /* 电流环 Ki */
    gpstMotorDefault->ol_start_phase_boost = M_PI / 2.0f; /* 仅 ol_align_enable=false 时兼容旧启动方式 */
    /*
     * 无感启动先预对齐再加速：0.5A Id 固定 0.3 秒。
     * 若电机只轻微摆动或对齐力不足，优先小步增加到 0.6A，而不是直接提高 Iq。
     */
    gpstMotorDefault->ol_align_enable  = true;
    gpstMotorDefault->ol_align_current = 0.5f;
    gpstMotorDefault->ol_align_time    = 0.3f;
    gpstMotorDefault->ol_align_angle   = 0.0f;
    /* 速度环默认关闭。先确认霍尔方向和电流环稳定，再用命令开启。 */
    gpstMotorDefault->speed_enable          = false;
    gpstMotorDefault->speed_allow_braking   = false;
    gpstMotorDefault->speed_kp              = 0.003f; /* 300RPM 误差产生约 0.9A P 输出 */
    gpstMotorDefault->speed_ki              = 0.01f;
    gpstMotorDefault->speed_iq_limit        = 2.0f;
    gpstMotorDefault->speed_loop_hz         = 1000.0f;
    /* 无感参数必须使用 Ω、H、Wb，不能直接填 mΩ、uH、mWb 数字。 */
    gpstMotorDefault->sensorless_enable          = true;
    gpstMotorDefault->sensorless_takeover        = false;
    gpstMotorDefault->sensorless_motor_r         = 0.4f;
    gpstMotorDefault->sensorless_motor_l         = 0.0008f;
    gpstMotorDefault->sensorless_flux_linkage    = 0.0091f;
    gpstMotorDefault->sensorless_observer_gain   = 12000000.0f;
    gpstMotorDefault->sensorless_pll_kp           = 2000.0f;
    gpstMotorDefault->sensorless_pll_ki           = 30000.0f;
    gpstMotorDefault->sensorless_transition_rpm  = 500.0f;
    gpstMotorDefault->sensorless_fallback_rpm    = 300.0f;
    /* 观测器未锁定时禁止 I/F 继续强拖到失步区；闭环后自动解除该上限。 */
    gpstMotorDefault->sensorless_openloop_max_rpm = 800.0f;
    gpstMotorDefault->sensorless_lock_time        = 0.10f;
    gpstMotorDefault->sensorless_loss_time        = 0.10f;
    gpstMotorDefault->sensorless_blend_time       = 0.20f;
    /* 允许最大约 80° 的 I/F 负载角，但仍禁止接近/超过 90° 的危险接管。 */
    gpstMotorDefault->sensorless_phase_tolerance = 1.40f;
    gpstMotorDefault->sensorless_phase_offset     = 0.0f;
    gpstMotorDefault->sensorless_speed_filter     = 0.10f;

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
    gpstMotor->ol_align_enable  = gpstMotorDefault->ol_align_enable;
    gpstMotor->ol_align_current = gpstMotorDefault->ol_align_current;
    gpstMotor->ol_align_time    = gpstMotorDefault->ol_align_time;
    gpstMotor->ol_align_angle   = gpstMotorDefault->ol_align_angle;
    /* 速度环调试参数同样从代码默认值覆盖，避免 Flash 旧值意外开启速度环。 */
    gpstMotor->speed_enable          = gpstMotorDefault->speed_enable;
    gpstMotor->speed_allow_braking   = gpstMotorDefault->speed_allow_braking;
    gpstMotor->speed_kp              = gpstMotorDefault->speed_kp;
    gpstMotor->speed_ki              = gpstMotorDefault->speed_ki;
    gpstMotor->speed_iq_limit        = gpstMotorDefault->speed_iq_limit;
    gpstMotor->speed_loop_hz         = gpstMotorDefault->speed_loop_hz;
    gpstMotor->sensorless_enable          = gpstMotorDefault->sensorless_enable;
    gpstMotor->sensorless_takeover        = gpstMotorDefault->sensorless_takeover;
    gpstMotor->sensorless_motor_r         = gpstMotorDefault->sensorless_motor_r;
    gpstMotor->sensorless_motor_l         = gpstMotorDefault->sensorless_motor_l;
    gpstMotor->sensorless_flux_linkage    = gpstMotorDefault->sensorless_flux_linkage;
    gpstMotor->sensorless_observer_gain   = gpstMotorDefault->sensorless_observer_gain;
    gpstMotor->sensorless_pll_kp           = gpstMotorDefault->sensorless_pll_kp;
    gpstMotor->sensorless_pll_ki           = gpstMotorDefault->sensorless_pll_ki;
    gpstMotor->sensorless_transition_rpm  = gpstMotorDefault->sensorless_transition_rpm;
    gpstMotor->sensorless_fallback_rpm    = gpstMotorDefault->sensorless_fallback_rpm;
    gpstMotor->sensorless_openloop_max_rpm = gpstMotorDefault->sensorless_openloop_max_rpm;
    gpstMotor->sensorless_lock_time        = gpstMotorDefault->sensorless_lock_time;
    gpstMotor->sensorless_loss_time        = gpstMotorDefault->sensorless_loss_time;
    gpstMotor->sensorless_blend_time       = gpstMotorDefault->sensorless_blend_time;
    gpstMotor->sensorless_phase_tolerance = gpstMotorDefault->sensorless_phase_tolerance;
    gpstMotor->sensorless_phase_offset     = gpstMotorDefault->sensorless_phase_offset;
    gpstMotor->sensorless_speed_filter     = gpstMotorDefault->sensorless_speed_filter;

    Motor_Speed_Init();
    Motor_Sensorless_Init();

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
