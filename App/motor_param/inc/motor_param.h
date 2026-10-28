/*
 * @Description: 电机运行参数与全局状态定义（兼容布局，按 config/command/runtime/calibration 分组注释）
 * @Author: lkc
 */
#ifndef _MOTOR_PARAM_H
#define _MOTOR_PARAM_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "motor_types.h"
#include "motor_nameplate.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MOTOR_OL_MODE_VF (0) /* V/F 电压开环 */
#define MOTOR_OL_MODE_IF (1) /* 角度开环 + 电流闭环 (I/F) */
/* Private macro -------------------------------------------------------------*/

/**
 * @description: 运行时状态快照（不持久化）。字段名与 MOTOR_PARAM_S 对应，经适配拷贝。
 * @detail: 不含配置增益、校准表与命令 enable/目标；Shell 不应直接写积分。
 */
typedef struct {
    float currentAdc[3];
    float voltAdc[3];
    float ia; /* A */
    float ib; /* A */
    float ic; /* A */
    float va; /* V */
    float vb; /* V */
    float vc; /* V */
    float v_bus; /* V */
    float Ialpha;
    float Ibeta;
    float Id; /* A */
    float Iq; /* A */
    float id_target; /* 本拍电流环 Id 目标 A */
    float iq_target; /* 本拍电流环 Iq 目标 A */
    float theta; /* 电角度 rad */
    float modD;
    float modQ;
    float modAlpha;
    float modBeta;
    uint32_t duty1;
    uint32_t duty2;
    uint32_t duty3;
    uint8_t curHallABC;
    uint8_t lastHallABC;
    float ol_current_rpm; /* 斜坡转速 RPM */
    float ol_theta;       /* 开环电角度 rad */
    float ol_vd_int;      /* d 轴积分 V */
    float ol_vq_int;      /* q 轴积分 V */
} motor_runtime_t;

/*
 * MOTOR_PARAM_S：兼容旧字段名与布局，禁止仅为改名而全仓替换。
 * 逻辑分组（持久化只走 motor_settings 的 config/calibration）：
 *   runtime      — 样本、调制、斜坡、积分
 *   command      — enable/mode/目标电流与转速、speed_enable
 *   config       — 增益、限幅、模型、对齐/HFI 等（见 motor_config_t）
 *   calibration  — 偏移与霍尔表
 */
typedef struct motor_param {
    /* ---- runtime: 采样与变换 ---- */
    float currentAdc[3];
    float voltAdc[3];

    float ia; /* 检测到的电流 a (A) */
    float ib; /* 检测到的电流 b (A) */
    float ic; /* 检测到的电流 c (A) */

    float va; /* 检测到的电压 a (V) */
    float vb; /* 检测到的电压 b (V) */
    float vc; /* 检测到的电压 c (V) */

    float v_bus; /* 母线电压 (V) */

    float Ialpha;
    float Ibeta;

    float Id; /* Park 变换 d 轴电流 (A) */
    float Iq; /* Park 变换 q 轴电流 (A) */
    float id_target; /* 本拍电流环 Id 目标 (A)，不持久化 */
    float iq_target; /* 本拍电流环 Iq 目标 (A)，不持久化 */

    float theta; /* 电角度 (rad) */

    /* 最近一次 Park 的电角度及其正余弦，同实例反 Park 角度原码相同时复用；仅控制上下文写。
     * 有效标志 park_sincos_valid 放在 ol_enable 后的对齐空洞里。 */
    float park_theta;      /* rad */
    float park_sin;
    float park_cos;

    /* ---- runtime: 调制与占空 ---- */
    float modD;
    float modQ;

    float modAlpha;
    float modBeta;

    uint32_t duty1;
    uint32_t duty2;
    uint32_t duty3;

    uint8_t curHallABC;
    uint8_t lastHallABC;
    uint8_t hallABC[6];

    /* ---- calibration ---- */
    float currentOffset[3];      /* 电流偏移 */
    float voltOffset[3];         /* 电压偏移 */
    float voltundrivenOffset[3]; /* 电压无负载偏移 */
    float encoder_offset_rad;    /* 绝对编码器电角偏置 rad */
    bool encoder_inverted;       /* 编码器方向取反 */
    bool encoder_calibrated;     /* 方向与零点已标定 */

    /* ---- command ---- */
    bool ol_enable; /* open-loop enable */
    /* ---- runtime ---- */
    bool park_sincos_valid; /* 占用 ol_mode 前的对齐空洞；RW_IRAM1 余量只有几十字节，不要挪 */
    /* ---- command ---- */
    int ol_mode;             /* MOTOR_OL_MODE_VF 或 MOTOR_OL_MODE_IF */
    float ol_target_rpm;     /* 目标转速 RPM */
    /* ---- config ---- */
    float ol_max_rpm;        /* max target speed RPM */
    /* ---- runtime ---- */
    float ol_current_rpm;    /* 当前斜坡转速 RPM */
    /* ---- config ---- */
    float ol_v_boost;        /* V/F 启动补偿 V */
    float ol_kv_f;           /* V/F 斜率 V/Hz */
    float ol_ramp_rpm_s;     /* 软启动速率 RPM/s */
    float ol_vq_limit;       /* Vq 上限 V */
    /* ---- runtime ---- */
    float ol_theta;          /* 开环电角度 rad */
    /* ---- config ---- */
    float ol_foc_freq;       /* ADC-DMA 基准频率 Hz */
    float ol_pole_pairs;     /* 电机极对数 */
    float ol_min_rpm;        /* 旧启动策略兼容参数；平滑启动不作转速地板 */
    /* ---- command ---- */
    float ol_iq_set;         /* 目标 Iq 电流 A */
    float ol_id_set;         /* 目标 Id 电流 A */
    float fw_current_max;    /* 弱磁 |Id| 上限 A，0=关闭 */
    float fw_duty_start;     /* 弱磁进入门槛，|V|/Vlim */
    float fw_ramp_a_s;       /* 弱磁 Id 斜坡 A/s */
    /* ---- config ---- */
    float ol_current_kp;     /* 开环电流环 Kp */
    float ol_current_ki;     /* 开环电流环 Ki */
    /* ---- runtime ---- */
    float ol_vd_int;         /* d 轴积分项 */
    float ol_vq_int;         /* q 轴积分项 */
    /* ---- config ---- */
    float ol_start_phase_boost; /* 兼容旧启动方式的一拍相位补偿 rad */
    bool ol_align_enable;    /* true=每次 I/F 启动前执行预对齐 */
    float ol_align_current;  /* 对齐 D 轴电流 Id (A) */
    float ol_align_time;     /* 固定磁场保持时间 (s) */
    float ol_align_angle;    /* 固定对齐电角度 (rad) */
    bool hfi_start_enable;   /* true=I/F 启动前先扫描转子凸极轴 */
    float hfi_start_voltage; /* 双极性注入电压幅值 V */
    uint16_t hfi_start_cycles;
    float hfi_start_min_confidence;

    /* ---- command / config（speed_enable 作命令开关；其余为配置） ---- */
    bool speed_enable;
    bool speed_allow_braking;
    bool position_hold;         /* 运行时：位置环使能。不写入 Flash */
    bool position_voltage_hold; /* 运行时：位置速度 PI 直接给 Vq，原速度环不参与 */
    float position_hold_v;      /* 运行时：位置速度 PI 电压 (V)，不含齿槽补偿 */
    float speed_kp;
    float speed_ki;
    float speed_iq_limit;
    float speed_loop_hz;

    bool sensorless_enable;
    bool sensorless_takeover;
    float sensorless_motor_r;
    float sensorless_motor_l;
    float sensorless_flux_linkage;
    float sensorless_observer_gain;
    float sensorless_pll_kp;
    float sensorless_pll_ki;
    float sensorless_transition_rpm;
    float sensorless_fallback_rpm;
    float sensorless_openloop_max_rpm;
    float sensorless_lock_time;
    float sensorless_loss_time;
    float sensorless_blend_time;
    float sensorless_phase_tolerance;
    float sensorless_phase_offset;
    float sensorless_speed_filter;

    /* ---- calibration ---- */
    int8_t hall_table[MOTOR_HALL_TABLE_SIZE]; /* -1 无效，0~200 为角度编码 */
    int hall_extra_samples;                   /* 霍尔 GPIO 额外滤波次数 */
    bool joint_voltage_hold;                  /* 运行时：关节阻抗直接给 Vq，位置环和速度环不参与 */
    float joint_hold_v;                       /* 运行时：关节阻抗合成电压 (V)，不含齿槽补偿 */
    bool joint_current_hold;                  /* 运行时：关节扭矩模式直接给 Iq，速度环不参与 */
    float joint_hold_iq;                      /* 运行时：关节扭矩模式 Iq 目标 (A) */
    /* 配置提交版本，不持久化；0 表示传统直接写入，ISR 仍逐字段检查。 */
    uint32_t control_config_revision;
} MOTOR_PARAM_S;

typedef struct tagFlashParam {
    MOTOR_PARAM_S motor;
} FLASH_PARAM_S;
/* Private variables ---------------------------------------------------------*/
extern MOTOR_PARAM_S motor;
extern MOTOR_PARAM_S *gpstMotor;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 电机参数初始化。
 * @detail: 写入固定参数与开环/电流/速度/无感默认值，尝试从 Flash 加载已保存参数，
 *          并初始化速度环与无感观测器。
 * @return {void} 无
 * @author: lkc
 */
void MOTOR_Param_Init(void);
#ifdef __cplusplus
}
#endif

#endif
