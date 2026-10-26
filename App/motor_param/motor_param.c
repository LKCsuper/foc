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
 * @return {void} 无
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

    /* 二、开环控制参数 */
    /*
     * 调试阶段上电后保持停机，避免刚烧录新算法就自动冲到高转速。
     * 需要转动时由 Shell 的 olspeed/olstart 明确启动。
     */
    gpstMotorDefault->ol_enable = false;
    /* 默认 I/F：低阻电机用电流限幅拖转；V/F 高压易过流。 */
    gpstMotorDefault->ol_mode = MOTOR_OL_MODE_IF;
    gpstMotorDefault->ol_target_rpm = 72.0f;              /* 默认开环目标转速 RPM */
    gpstMotorDefault->ol_max_rpm = MOTOR_FIXED_RATED_RPM; /* 不超过额定转速 */
    /*
     * V/F 启动补偿电压按所选电机配置取默认值；运行电压由补偿电压与 V/F 斜率
     * 共同计算，并经过软启动及电压限幅，不等同于固定 Vq 拖转。
     * ol_vq_limit 同时用于控制输出限幅，运行时还受 Vbus/sqrt(3) 限制。
     */
    gpstMotorDefault->ol_v_boost = MOTOR_OPEN_LOOP_VBOOST_DEFAULT;
    gpstMotorDefault->ol_kv_f = 0.057f;                   /* V/Hz */
    gpstMotorDefault->ol_ramp_rpm_s = 10.0f;              /* I/F、V/F 开环转速斜坡 RPM/s */
    gpstMotorDefault->ol_vq_limit = MOTOR_OPEN_LOOP_VQ_LIMIT_DEFAULT; /* 运行时再受 Vbus/sqrt(3) 限制 */
    gpstMotorDefault->ol_foc_freq = 30000.0f;             /* ADC-DMA 基准频率回退值；优先读取 HAL，实际 FOC 频率还需除以分频系数 */
    gpstMotorDefault->ol_min_rpm = 72.0f;                 /* olstart 在目标为零时的默认转速；正常平滑启动不作转速地板 */
    /* 开环电流模式参数 */
    gpstMotorDefault->ol_iq_set = MOTOR_OPEN_LOOP_IQ_DEFAULT;
    gpstMotorDefault->ol_id_set = 0.0f;                   /* 目标 Id 电流 A */
    /*
     * 弱磁原理：转速升高时，q 轴反电势项约为 ωe*(ψf + Ld*Id)。所需电压接近
     * 逆变器输出上限后，电流环的调节余量减小；施加负 Id 可减小 d 轴等效磁链，
     * 降低该反电势项，为电压受限时继续提速留出余量。
     * 负 Id 会增加铜耗，并占用电流圆中供 Iq 产生转矩的额度，还受永磁体退磁
     * 限制；允许的弱磁电流必须按具体电机确定，因此默认电流上限为 0，即关闭。
     * 当前按电压利用率 sqrt(Vd²+Vq²)/Vlim 触发，不按固定转速触发；Vlim 同时
     * 受配置限幅与母线电压约束。Vbus/√3 来自线性 SVPWM 的电压矢量几何关系：
     * 在幅值不变的 Clarke/Park 定义下，两电平逆变器的六个有效电压矢量长度均为
     * 2*Vbus/3，构成正六边形；要让旋转电压矢量在所有角度都能线性合成，其轨迹
     * 必须位于六边形内切圆内，因此最大幅值为 (2*Vbus/3)*cos(30°)=Vbus/√3。
     * 这里的幅值指 sqrt(Vd²+Vq²)，对应平衡正弦相电压的峰值，并非线电压有效值；
     * 超过内切圆后，部分角度无法合成所需电压，进入过调制，电流环调节会受限。
     * 利用率达到 fw_duty_start 时，Id 向负电流上限爬升；降至门槛以下 0.05 时
     * 向 0 恢复，中间区间保持，形成滞环以避免反复切换。fw_ramp_a_s 限制 Id
     * 变化速度，减轻电流与转矩突变；门槛和斜坡需结合具体电机及电流环响应调试。
     */
    gpstMotorDefault->fw_current_max = 0.0f;             /* 弱磁 |Id| 上限 A，0=关闭 */
    gpstMotorDefault->fw_duty_start = 0.80f;             /* 进入弱磁的电压利用率门槛 */
    gpstMotorDefault->fw_ramp_a_s = 8.0f;                /* 弱磁 Id 变化速率 A/s */
    /*
     * 电流环将采样电流变换到旋转 d/q 坐标系，分别比较目标与实测 Id/Iq，
     * 由两轴 PI 输出 Vd/Vq，再经逆变换和 SVPWM 驱动电机，使电流跟随目标。
     * Kp 按当前电流误差立即修正电压，Ki 累积误差以消除稳态偏差；增益过大
     * 会降低稳定裕量，引起过冲或振荡，过小则跟踪迟缓。
     * 忽略轴间耦合与反电势时，单轴电流对象近似为 1/(L*s+R)，可按目标带宽
     * ωc 估算 Kp≈L*ωc、Ki≈R*ωc（连续 PI：Kp*e + Ki*∫e dt）。实际参数还需
     * 结合采样/控制周期、滤波与计算延迟、调制增益及电压限幅验证。
     * 默认增益由所选电机配置提供，不以某台电机的速度过冲或历史倍率统一折算。
     */
    gpstMotorDefault->ol_current_kp = MOTOR_CURRENT_KP_DEFAULT;
    gpstMotorDefault->ol_current_ki = MOTOR_CURRENT_KI_DEFAULT;
    /* 旧开环启动角度偏置，与电流 PI 增益无关；仅 ol_align_enable=false 时使用。 */
    gpstMotorDefault->ol_start_phase_boost = M_PI / 2.0f; /* 90° 电角度 */
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

    PRINTF("Motor iq set [%f]\r\n", gpstMotor->ol_iq_set);

    return;
}
#ifdef __cplusplus
}
#endif
