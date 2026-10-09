/*
 * @Description: 电机运行参数初始化与全局状态
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "motor_param.h"
#include "flash_storage.h"
#include "motor_math.h"
#include "motor_speed.h"
#include "motor_sensorless.h"
#include "printf_color.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
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
 *          加载已保存参数，失败则使用默认值且不自动擦写 Flash；随后用代码默认值覆盖运行参数，
 *          并初始化速度环与无感观测器。R/L/磁链检测结果暂不应用。
 * @return {*}
 * @author: lkc
 */
void MOTOR_Param_Init(void)
{
    bool flash_ok;
    motor_settings_t defaults;
    motor_settings_t loaded;

    /* 重入初始化也不得继承上次的命令、采样或 PI 状态。 */
    memset(gpstMotorDefault, 0, sizeof(*gpstMotorDefault));

    /* 一、电机固定参数：铭牌见 motor_nameplate.h */
    gpstMotorDefault->ol_pole_pairs = MOTOR_FIXED_POLE_PAIRS;

    /* 二、运行给定值：属于控制指令，不是电机固有参数 */
    gpstMotorDefault->e_iq_set = 1.5f;

    /* 三、开环控制参数 */
    /*
     * 调试阶段上电后保持停机，避免刚烧录新算法就自动冲到高转速。
     * 需要转动时由 Shell 的 olspeed/olstart 明确启动。
     */
    gpstMotorDefault->ol_enable = false;
    /* 默认 I/F：低阻电机用电流限幅拖转；V/F 高压易过流。 */
    gpstMotorDefault->ol_mode = MOTOR_OL_MODE_IF;
    gpstMotorDefault->ol_target_rpm = 72.0f;              /* 对齐已上板成功的旧开环等效转速 */
    gpstMotorDefault->ol_max_rpm = MOTOR_FIXED_RATED_RPM; /* 不超过额定转速 */
    /*
     * 旧开环基线 3e5d564 在当前 24 V 电机/功率板上以固定 Vq=5 V、约 72 RPM
     * 能稳定旋转。1.5 V 实测只能令转子往返摆动，因此先恢复已验证的启动强度；
     * 仍由 ol_vq_limit 与 Vbus/sqrt(3) 统一限幅，后续上板再向下寻找最低可靠值。
     */
    gpstMotorDefault->ol_v_boost = MOTOR_OPEN_LOOP_VBOOST_DEFAULT;
    gpstMotorDefault->ol_kv_f = 0.057f;                   /* V/Hz */
    gpstMotorDefault->ol_ramp_rpm_s = 10.0f;              /* 软启动速率 — 越慢越稳 */
    gpstMotorDefault->ol_vq_limit = MOTOR_OPEN_LOOP_VQ_LIMIT_DEFAULT; /* 运行时再受 Vbus/sqrt(3) 限制 */
    gpstMotorDefault->ol_foc_freq = 30000.0f;             /* ADC-DMA 基准频率；实际 FOC 频率还要除以分频系数 */
    gpstMotorDefault->ol_min_rpm = 72.0f;                 /* HD75 实机验证的最低稳定开环拖动转速 */
    /* 开环电流模式参数 */
    gpstMotorDefault->ol_iq_set = MOTOR_OPEN_LOOP_IQ_DEFAULT;
    gpstMotorDefault->ol_id_set = 0.0f;                   /* 目标 Id 电流 A */
    /*
     * 线性 SVPWM 下 Vlim≈Vbus/√3。本电机 3000 RPM 时 ωeλ≈16.2 V，Id=0 会顶满电压。
     * 台架电源只有 3 A；2026-09-16 实测 1.5 A 弱磁未能跟上 1500 RPM，且 Id 有过冲。
     * 默认关闭，待排查 Hall 角度/电流跟踪后再通过 fw 命令受控启用。
     */
    gpstMotorDefault->fw_current_max = 0.0f;
    gpstMotorDefault->fw_duty_start = 0.80f;
    gpstMotorDefault->fw_ramp_a_s = 8.0f;
    /*
     * foc_svm 把同一 Vd/Vq 的实际相电压从约 1/3 拉到满幅。
     * Kp/Ki 按 Hall CCR 1.70 时的电流带宽折算（0.2×1.70/3、30×1.70/3），
     * 150 RPM 上 olkp 0.2 会过冲到约 210 RPM。
     */
    gpstMotorDefault->ol_current_kp = MOTOR_CURRENT_KP_DEFAULT;
    gpstMotorDefault->ol_current_ki = MOTOR_CURRENT_KI_DEFAULT;
    gpstMotorDefault->ol_start_phase_boost = M_PI / 2.0f; /* 仅 ol_align_enable=false 时兼容旧启动方式 */
    /*
     * 默认关对齐/HFI：开环先拖转；切 I/F 后再按需 olalign / hfistart。
     */
    gpstMotorDefault->ol_align_enable = false;
    gpstMotorDefault->ol_align_current = MOTOR_ALIGN_CURRENT_DEFAULT;
    gpstMotorDefault->ol_align_time = 0.3f;
    gpstMotorDefault->ol_align_angle = 0.0f;
    gpstMotorDefault->hfi_start_enable = false;
    gpstMotorDefault->hfi_start_voltage = 2.0f;
    gpstMotorDefault->hfi_start_cycles = 8U;
    gpstMotorDefault->hfi_start_min_confidence = 0.05f;
    /* 速度环默认关闭。先确认霍尔方向和电流环稳定，再用命令开启。 */
    gpstMotorDefault->speed_enable = false;
    gpstMotorDefault->speed_allow_braking = false;
    gpstMotorDefault->speed_kp = MOTOR_SPEED_KP_DEFAULT;
    gpstMotorDefault->speed_ki = MOTOR_SPEED_KI_DEFAULT;
    gpstMotorDefault->speed_iq_limit = MOTOR_SPEED_IQ_LIMIT_DEFAULT;
    gpstMotorDefault->speed_loop_hz = 1000.0f;
    /* 无感电机上电就积分观测器。接管仍默认关闭，电角先用 I/F。 */
    gpstMotorDefault->sensorless_enable = (MOTOR_SENSOR_TYPE == MOTOR_SENSOR_SENSORLESS);
    gpstMotorDefault->sensorless_takeover = false;
    /* 电阻、电感、磁链单位是 Ω、H、Wb。 */
    gpstMotorDefault->sensorless_motor_r = MOTOR_FIXED_LINE_RESISTANCE;
    gpstMotorDefault->sensorless_motor_l = MOTOR_FIXED_LINE_INDUCTANCE;
    gpstMotorDefault->sensorless_flux_linkage = MOTOR_FIXED_FLUX_LINKAGE;
    gpstMotorDefault->sensorless_observer_gain = 12000000.0f;
    gpstMotorDefault->sensorless_pll_kp = 2000.0f;
    gpstMotorDefault->sensorless_pll_ki = 30000.0f;
    gpstMotorDefault->sensorless_transition_rpm = 500.0f;
    gpstMotorDefault->sensorless_fallback_rpm = 300.0f;
    /* 观测器未锁定时禁止 I/F 继续强拖到失步区；闭环后自动解除该上限。 */
    gpstMotorDefault->sensorless_openloop_max_rpm = 800.0f;
    gpstMotorDefault->sensorless_lock_time = 0.10f;
    gpstMotorDefault->sensorless_loss_time = 0.10f;
    gpstMotorDefault->sensorless_blend_time = 0.20f;
    /* 允许最大约 80° 的 I/F 负载角，但仍禁止接近/超过 90° 的危险接管。 */
    gpstMotorDefault->sensorless_phase_tolerance = 1.40f;
    gpstMotorDefault->sensorless_phase_offset = 0.0f;
    gpstMotorDefault->sensorless_speed_filter = 0.10f;

    gpstMotorDefault->hall_extra_samples = 1;
    {
        int hall_i;

        for (hall_i = 0; hall_i < (int)MOTOR_HALL_TABLE_SIZE; hall_i++) {
            gpstMotorDefault->hall_table[hall_i] = MOTOR_HALL_TABLE_INVALID;
        }
    }

    /* 先建立完整默认对象，再仅加载配置/校准，绝不反序列化运行状态。 */
    *gpstMotor = *gpstMotorDefault;

    /* 尝试从 Flash 加载已保存的电机配置与校准 */
    flash_ok = FlashStorage_Load(gpstMotor);
    if (!flash_ok) {
        /* 首次上电、旧格式或数据损坏：保留代码默认值 */
        /* 不在上电自动擦写：保留旧版本槽，用户校准后用 saveparam 显式保存。 */
        PRINTF("Default Flash Param\r\n");
    }
    /* 保留当前开发策略：控制增益用代码默认值，校准与霍尔采样配置从 Flash 恢复。
     * 通过独立配置对象覆盖，避免逐个字段复制时漏清运行状态或漏掉新增配置。 */
    if (Motor_Settings_Capture(gpstMotorDefault, &defaults) && Motor_Settings_Capture(gpstMotor, &loaded)) {
        defaults.calibration = loaded.calibration;
        defaults.config.hall_extra_samples = loaded.config.hall_extra_samples;
        if (!Motor_Settings_Apply(gpstMotor, &defaults)) {
            *gpstMotor = *gpstMotorDefault;
            PRINTF("Motor settings: apply failed, using defaults\r\n");
        }
    } else {
        *gpstMotor = *gpstMotorDefault;
        PRINTF("Motor settings: invalid defaults or calibration\r\n");
    }

    Motor_Speed_Init();
    Motor_Sensorless_Init();

    PRINTF("Motor iq set [%f]\r\n", gpstMotor->e_iq_set);

    return;
}
#ifdef __cplusplus
}
#endif
