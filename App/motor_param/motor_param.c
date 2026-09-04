/*
 * @Description: 电机运行参数初始化与全局状态
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* ATK-BL57H95D20E 电机固定参数（来自电机铭牌/规格书） */
#define MOTOR_FIXED_MODEL               "ATK-BL57H95D20E" /* 型号；当前未使用 */
#define MOTOR_FIXED_PHASE_COUNT         (3U)        /* 相数；当前未使用 */
#define MOTOR_FIXED_POLE_PAIRS          (2.0f)      /* 4 极 = 2 对极 */
#define MOTOR_FIXED_RATED_VOLTAGE       (24.0f)     /* 额定电压 VDC；当前未使用 */
#define MOTOR_FIXED_RATED_RPM           (3000.0f)   /* 额定转速 RPM */
#define MOTOR_FIXED_RATED_TORQUE        (0.33f)     /* 额定扭矩 N·m；当前未使用 */
#define MOTOR_FIXED_RATED_CURRENT       (6.6f)      /* 额定电流 A；当前未使用 */
#define MOTOR_FIXED_OUTPUT_POWER        (105.0f)    /* 输出功率 W；当前未使用 */
#define MOTOR_FIXED_PEAK_TORQUE         (4.2f)      /* 峰值扭矩 N·m；当前未使用 */
#define MOTOR_FIXED_PEAK_CURRENT        (19.8f)     /* 峰值电流 A；当前未使用 */
#define MOTOR_FIXED_TORQUE_CONSTANT     (0.063f)    /* 力矩常数 N·m/A；当前未使用 */
#define MOTOR_FIXED_BACK_EMF            (6.6f)      /* 反电势 V/kRPM；当前未使用 */
#define MOTOR_FIXED_LINE_RESISTANCE     (0.4f)      /* 线电阻 Ω */
#define MOTOR_FIXED_LINE_INDUCTANCE     (0.0008f)   /* 线电感 0.8 mH = 0.0008 H */
#define MOTOR_FIXED_INERTIA             (210.0f)    /* 转动惯量 g·cm^2；当前未使用 */
#define MOTOR_FIXED_BODY_LENGTH         (95.0f)     /* 机身长度 mm；当前未使用 */
#define MOTOR_FIXED_WEIGHT              (1.0f)      /* 重量 kg；当前未使用 */

/* 按线间 RMS 反电势计算：λ = Ke / (128.25498 × 极对数) */
#define MOTOR_FIXED_FLUX_LINKAGE        (0.02573f)  /* 6.6 V/kRPM、2 极对 -> 25.73 mWb */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
MOTOR_PARAM_S motor;
static MOTOR_PARAM_S motorDefault;
MOTOR_PARAM_S *gpstMotor = &motor;
static MOTOR_PARAM_S *gpstMotorDefault = &motorDefault;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 电机参数初始化
 * @detail: 依次写入电机固定参数与开环/电流/速度/无感等运行默认值；尝试从 Flash
 *          加载已保存参数，失败则以默认值写入 Flash；随后用代码默认值覆盖运行参数，
 *          并初始化速度环与无感观测器。R/L/磁链检测结果暂不应用。
 * @return {*}
 * @author: lkc
 */
VOID MOTOR_Param_Init(VOID)
{
    motor_detect_result_t detect_buf;
    bool flash_ok;

    /* 一、电机固定参数：更换电机时按铭牌/辨识结果修改 */
    gpstMotorDefault->ol_pole_pairs = MOTOR_FIXED_POLE_PAIRS;

    /* 二、运行给定值：属于控制指令，不是电机固有参数 */
    gpstMotorDefault->e_iq_set = 1.5f;

    /* 三、开环控制参数 */
    /*
     * 调试阶段上电后保持停机，避免刚烧录新算法就自动冲到高转速。
     * 需要转动时由 Shell 的 olspeed/olstart 明确启动。
     */
    gpstMotorDefault->ol_enable      = false;
    gpstMotorDefault->ol_mode        = MOTOR_OL_MODE_IF;
    gpstMotorDefault->ol_target_rpm  = 500.0f;     /* 首次手动 olstart 的安全目标转速 RPM */
    gpstMotorDefault->ol_max_rpm     = MOTOR_FIXED_RATED_RPM; /* 不超过额定转速 */
    gpstMotorDefault->ol_v_boost     = 1.0f;    /* 启动补偿 V */
    gpstMotorDefault->ol_kv_f        = 0.057f;   /* V/Hz */
    gpstMotorDefault->ol_ramp_rpm_s  = 50.0f;    /* 软启动速率 — 越慢越稳 */
    gpstMotorDefault->ol_vq_limit    = 18.0f;   /* 最大 75% 调制 */
    gpstMotorDefault->ol_foc_freq    = 30000.0f; /* ADC-DMA 基准频率；实际 FOC 频率还要除以分频系数 */
    gpstMotorDefault->ol_min_rpm     = 30.0f;    /* 最低转速地板 RPM */
    /* 开环电流模式参数 */
    gpstMotorDefault->ol_iq_set      = 0.3f;     /* 首次 I/F 起步目标 Iq 电流 A */
    gpstMotorDefault->ol_id_set      = 0.0f;     /* 目标 Id 电流 A */
    gpstMotorDefault->ol_current_kp  = 0.2f;     /* 电流环 Kp */
    gpstMotorDefault->ol_current_ki  = 30.0f;    /* 电流环 Ki */
    gpstMotorDefault->ol_start_phase_boost = M_PI / 2.0f; /* 仅 ol_align_enable=false 时兼容旧启动方式 */
    /*
     * 无感启动先预对齐再加速：0.5A Id 固定 0.3 秒。
     * 若电机只轻微摆动或对齐力不足，优先小步增加到 0.6A，而不是直接提高 Iq。
     */
    gpstMotorDefault->ol_align_enable  = true;
    gpstMotorDefault->ol_align_current = 0.5f;
    gpstMotorDefault->ol_align_time    = 0.3f;
    gpstMotorDefault->ol_align_angle   = 0.0f;
    gpstMotorDefault->hfi_start_enable = true;
    gpstMotorDefault->hfi_start_voltage = 2.0f;
    gpstMotorDefault->hfi_start_cycles = 8U;
    gpstMotorDefault->hfi_start_min_confidence = 0.05f;
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
    gpstMotorDefault->sensorless_motor_r         = MOTOR_FIXED_LINE_RESISTANCE;
    gpstMotorDefault->sensorless_motor_l         = MOTOR_FIXED_LINE_INDUCTANCE;
    gpstMotorDefault->sensorless_flux_linkage    = MOTOR_FIXED_FLUX_LINKAGE;
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

    gpstMotorDefault->hall_extra_samples = 1;
    {
        int hall_i;

        for (hall_i = 0; hall_i < (int)MOTOR_HALL_TABLE_SIZE; hall_i++) {
            gpstMotorDefault->hall_table[hall_i] = MOTOR_HALL_TABLE_INVALID;
        }
    }

    /* 尝试从 Flash 加载已保存的电机参数和检测结果 */
    flash_ok = FlashStorage_Load(gpstMotor, &detect_buf);
    if (!flash_ok)
    {
        /* 首次上电或数据损坏：以代码默认值初始化并写入 Flash */
        memset(&detect_buf, 0, sizeof(detect_buf));
        detect_buf.version = MOTOR_DETECT_RESULT_VERSION;

        if (!FlashStorage_Save(gpstMotorDefault, &detect_buf))
        {
            ERROR("FlashStorage: save defaults failed\r\n");
        }
        PRINTF("Default Flash Param\r\n");
    }
    (void)detect_buf;

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
    gpstMotor->hfi_start_enable = gpstMotorDefault->hfi_start_enable;
    gpstMotor->hfi_start_voltage = gpstMotorDefault->hfi_start_voltage;
    gpstMotor->hfi_start_cycles = gpstMotorDefault->hfi_start_cycles;
    gpstMotor->hfi_start_min_confidence = gpstMotorDefault->hfi_start_min_confidence;
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
    if (!flash_ok) {
        gpstMotor->hall_extra_samples = gpstMotorDefault->hall_extra_samples;
        memcpy(gpstMotor->hall_table, gpstMotorDefault->hall_table, sizeof(gpstMotor->hall_table));
    }

    Motor_Speed_Init();
    Motor_Sensorless_Init();

    PRINTF("Motor iq set [%f]\r\n", gpstMotor->e_iq_set);

    return;
}
#ifdef __cplusplus
}
#endif
