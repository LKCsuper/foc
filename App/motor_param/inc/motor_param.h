/*
 * @Description: 电机运行参数与全局状态定义
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
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

typedef struct motor_param {
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

    float theta; /* 电角度 (rad) */

    /* 电流环给定（遗留字段，主路径优先用 ol_*） */
    float e_id_set;
    float e_iq_set;
    float e_err_ki_sum; /* 积分累加 */
    float e_ki_windup;  /* 积分抗饱和 */
    float e_iq_ki;      /* q 轴电流环 ki */
    float e_iq_kp;      /* q 轴电流环 kp */
    float e_id_ki;      /* d 轴电流环 ki */
    float e_id_kp;      /* d 轴电流环 kp */
    float e_iq_limit;   /* q 轴限制 */
    float e_id_limit;   /* d 轴限制 */

    float e_iq_out; /* q 轴实际输出 */
    float e_id_out; /* d 轴实际输出 */

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

    /* 采样零点 */
    float currentOffset[3];      /* 电流偏移 */
    float voltOffset[3];         /* 电压偏移 */
    float voltundrivenOffset[3]; /* 电压无负载偏移 */

    /* 开环控制：设 ol_target_rpm 即可，其余自动计算 */
    bool ol_enable; /* open-loop enable */
#define MOTOR_OL_MODE_VF (0) /* V/F 电压开环 */
#define MOTOR_OL_MODE_IF (1) /* 角度开环 + 电流闭环 (I/F) */
    int ol_mode;             /* MOTOR_OL_MODE_VF 或 MOTOR_OL_MODE_IF */
    float ol_target_rpm;     /* 目标转速 RPM */
    float ol_max_rpm;        /* max target speed RPM */
    float ol_current_rpm;    /* 当前斜坡转速 RPM */
    float ol_v_boost;        /* V/F 启动补偿 V */
    float ol_kv_f;           /* V/F 斜率 V/Hz */
    float ol_ramp_rpm_s;     /* 软启动速率 RPM/s */
    float ol_vq_limit;       /* Vq 上限 V */
    float ol_theta;          /* 开环电角度 rad */
    float ol_foc_freq;       /* ADC-DMA 基准频率 Hz */
    float ol_pole_pairs;     /* 电机极对数 */
    float ol_min_rpm;        /* 最低转速地板 RPM */
    float ol_iq_set;         /* 目标 Iq 电流 A */
    float ol_id_set;         /* 目标 Id 电流 A */
    float ol_current_kp;     /* 开环电流环 Kp */
    float ol_current_ki;     /* 开环电流环 Ki */
    float ol_vd_int;         /* d 轴积分项 */
    float ol_vq_int;         /* q 轴积分项 */
    float ol_start_phase_boost; /* 兼容旧启动方式的一拍相位补偿 rad */
    bool ol_align_enable;    /* true=每次 I/F 启动前执行预对齐 */
    float ol_align_current;  /* 对齐 D 轴电流 Id (A) */
    float ol_align_time;     /* 固定磁场保持时间 (s) */
    float ol_align_angle;    /* 固定对齐电角度 (rad) */
    bool hfi_start_enable;   /* true=I/F 启动前先扫描转子凸极轴 */
    float hfi_start_voltage; /* 双极性注入电压幅值 V */
    uint16_t hfi_start_cycles;
    float hfi_start_min_confidence;

    bool speed_enable;
    bool speed_allow_braking;
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

    int8_t hall_table[MOTOR_HALL_TABLE_SIZE]; /* -1 无效，0~200 为角度编码 */
    int hall_extra_samples;                   /* 霍尔 GPIO 额外滤波次数 */
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
 * @return {*}
 * @author: lkc
 */
void MOTOR_Param_Init(void);
#ifdef __cplusplus
}
#endif

#endif
