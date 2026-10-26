/*
 * @Description: Hall I/F 电流诊断（olcurdiag）：扇区累计、最差帧、换向窗与 mean/RMS
 * @Author: lkc
 */
#ifndef MOTOR_HALL_DIAG_H
#define MOTOR_HALL_DIAG_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
/* Private define ------------------------------------------------------------*/
#define MOTOR_HALL_DIAG_DIVIDER 16U          /* 扇区统计每 16 个控制拍写一次 */
#define MOTOR_HALL_DIAG_PHASE_SUM_A 0.5f     /* |Ia+Ib+Ic| 观察门槛 (A) */
#define MOTOR_HALL_DIAG_ID_ERROR_A 1.0f      /* |Id−目标| 观察门槛 (A) */
/* Hall 每 MOTOR_HALL_ADC_DIVIDER 拍才更新，换向后 sector_timer 保持 0 直到下次更新（约 0.27 ms）。 */
#define MOTOR_HALL_DIAG_EDGE_AGE_S 0.00015f
/* Private typedef -----------------------------------------------------------*/
/**
 * @description: Hall I/F 按扇区累计的电流诊断（每 16 个控制拍一次）
 */
typedef struct {
    uint32_t samples;                    /* 该扇区抽样次数 */
    uint32_t phase_sum_over;             /* |Ia+Ib+Ic| > 0.5 A 次数 */
    uint32_t phase_sum_positive_over;    /* 三相和 > +0.5 A 次数 */
    uint32_t phase_sum_negative_over;    /* 三相和 < −0.5 A 次数 */
    int64_t signed_phase_sum_milliamps;  /* 三相和累加 (mA)，用于均值 */
    uint64_t current_adc_sum[3];         /* CURR1/2/3 原始计数累加 */
    uint32_t id_error_over;              /* |Id−目标| > 1.0 A 次数 */
    uint32_t first_half_samples;         /* 估算扇区前半抽样次数 */
    uint32_t first_half_id_over;         /* 前半 |Id| 越界次数 */
    uint32_t second_half_samples;        /* 估算扇区后半抽样次数 */
    uint32_t second_half_id_over;        /* 后半 |Id| 越界次数 */
    float max_abs_phase_sum_a;           /* |Ia+Ib+Ic| 峰值 (A) */
    float max_abs_id_error_a;            /* |Id−目标| 峰值 (A) */
} motor_hall_current_diag_sector_t;

/**
 * @description: 本轮 |Ia+Ib+Ic| 最大那一拍的锁存快照
 */
typedef struct {
    float signed_sum_a;      /* 三相电流代数和 (A) */
    float abs_sum_a;         /* |Ia+Ib+Ic| (A) */
    float ia;                /* A 相电流 (A) */
    float ib;                /* B 相电流 (A) */
    float ic;                /* C 相电流 (A) */
    float id;                /* d 轴电流 (A) */
    float iq;                /* q 轴电流 (A) */
    uint16_t current_adc[3]; /* CURR1/2/3 原始 ADC 计数 */
    uint32_t duty[3];        /* 软件 A/B/C PWM 比较值（计数） */
    uint16_t tim1_cnt;       /* ISR 入口 TIM1 计数 */
    uint16_t tim1_arr;       /* ISR 入口 TIM1 ARR（计数） */
    uint8_t hall_state;      /* Hall 状态 0..7 */
    uint8_t recon_phase;     /* 本拍重构相：0=无，1=A，2=B，3=C */
    uint8_t first_half;      /* 1=扇区前半，0=后半 */
    float edge_age_ms;       /* 距上次 Hall 边沿 (ms) */
    float theta_rad;         /* 本拍 Park 用电角 (rad) */
    float sector_center_rad; /* 当前 Hall 扇区中心 (rad) */
    float id_target;         /* 本拍 Id 目标 (A) */
    float id_error;          /* |Id−目标| (A) */
    float prev_ia;           /* 换向前一拍 A 相电流 (A) */
    float prev_ib;           /* 换向前一拍 B 相电流 (A) */
    float prev_ic;           /* 换向前一拍 C 相电流 (A) */
    uint32_t beats;          /* 本轮已观察的 Hall FOC 拍数 */
} motor_hall_current_worst_t;

/**
 * @description: 霍尔换向窗与扇区内部的 |Id| 越界对照
 */
typedef struct {
    uint32_t edge_samples;               /* 边沿年龄≈0 的拍数（霍尔分频冻结窗） */
    uint32_t edge_id_over;               /* 上述拍中 |Id−目标| > 1 A */
    uint32_t body_samples;               /* 其余拍数 */
    uint32_t body_id_over;               /* 其余拍中 |Id−目标| > 1 A */
    uint32_t jump_samples;               /* 霍尔状态刚变化的拍数 */
    int64_t id_mA_sum;                   /* Id 带符号累加 (mA) */
    int64_t iq_mA_sum;                   /* Iq 带符号累加 (mA) */
    int64_t iq_target_mA_sum;            /* Iq 目标累加 (mA) */
    int64_t ia_mA_sum;                   /* Ia 带符号累加 (mA) */
    int64_t ib_mA_sum;                   /* Ib 带符号累加 (mA) */
    int64_t ic_mA_sum;                   /* Ic 带符号累加 (mA) */
    uint64_t id_abs_mA_sum;              /* |Id| 累加 (mA) */
    uint64_t iq_abs_mA_sum;              /* |Iq| 累加 (mA) */
    uint64_t id2_mA2_sum;                /* Id² 累加 (mA²)，用于 RMS */
    uint64_t iq2_mA2_sum;                /* Iq² 累加 (mA²)，用于 RMS */
    motor_hall_current_worst_t jump;     /* 换向拍里 |Id| 最大的一帧 */
} motor_hall_current_edge_diag_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 在 ADC-DMA ISR 入口记下 TIM1 计数，供最差帧对照采样沿。
 * @param {uint16_t} tim1_cnt  ISR 入口 TIM1 计数
 * @param {uint16_t} tim1_arr  ISR 入口 TIM1 ARR（计数）
 * @return {void} 无
 * @author: lkc
 */
void Motor_Hall_DiagNoteTimer(uint16_t tim1_cnt, uint16_t tim1_arr);

/**
 * @description: Hall I/F 稳态时累计三相和与 Id 跟踪诊断。
 * @detail: 每拍刷新最差帧；每 16 拍写入扇区统计。目标转速变化时清零。
 *          |sum|>0.5 A 与 |Id 误差|>1.0 A 只计数，不在本函数内停机。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Hall_DiagUpdate(void);

/**
 * @description: 复制 Hall 电流诊断的 8 个扇区累计快照。
 * @detail: 短关 IRQ 拷贝；停机后仍保留上一轮目标转速下的统计。
 * @param {motor_hall_current_diag_sector_t} sectors[8] 输出缓冲，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Hall_DiagGetSectors(motor_hall_current_diag_sector_t sectors[8]);

/**
 * @description: 复制本轮 |Ia+Ib+Ic| 最大那一拍的锁存快照。
 * @param {motor_hall_current_worst_t} *out 输出快照，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Hall_DiagGetWorst(motor_hall_current_worst_t *out);

/**
 * @description: 复制本轮 |Id−目标| 最大那一拍的锁存快照。
 * @param {motor_hall_current_worst_t} *out 输出快照，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Hall_DiagGetWorstId(motor_hall_current_worst_t *out);

/**
 * @description: 复制霍尔换向窗与扇区内部的 |Id| 越界对照。
 * @param {motor_hall_current_edge_diag_t} *out 输出快照，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Hall_DiagGetEdge(motor_hall_current_edge_diag_t *out);
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
