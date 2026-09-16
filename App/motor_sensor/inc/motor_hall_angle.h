/*
 * @Description: 霍尔表正常解码（扇区→电角、转速与非法状态）
 * @Author: lkc
 *
 * 学习要点：
 *   1) hall_table[i]：0..200 = 电角编码（deg×200/360），-1 无效（0/7 常为无效）。
 *   2) 表未就绪或当前扇区非法 → angle_valid=false，勿当有效零速闭环。
 *   3) 速度环用 Hall-PLL（ODrive 临界阻尼，cpr=6 电扇区）；六边沿窗口仅供显示。
 *   4) 低于 MOTOR_HALL_INTERP_START_RPM 电角钉在扇区中心（VESC 低速关插值）。
 */
#ifndef MOTOR_HALL_ANGLE_H
#define MOTOR_HALL_ANGLE_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "motor_types.h"
/* Private define ------------------------------------------------------------*/
#define MOTOR_HALL_ANGLE_STALL_S    (0.2f)     /* 无换向超过该时间视为停转 (s) */
#define MOTOR_HALL_SPEED_WINDOW     (6U)       /* 最近边沿周期窗口，供显示滤波 */
#define MOTOR_HALL_PLL_CPR          (6.0f)     /* 每个电周期 6 个霍尔扇区 */
#define MOTOR_HALL_PLL_BW_HZ        (10.0f)    /* ODrive 式 PLL 带宽；Hall 不可用编码器默认 1000 */
#define MOTOR_HALL_PLL_VEL_MAX      (4000.0f)  /* PLL 速度上限 (counts/s)，约 2 对极 2 万 RPM */
#define MOTOR_HALL_PLL_ENTER_RPM    (200.0f)   /* 低于此转速从周期测速切入 PLL */
#define MOTOR_HALL_PLL_EXIT_RPM     (300.0f)   /* 高于此转速沿用已验证的六边沿测速 */
#define MOTOR_HALL_MIN_PERIOD_S     (2.0e-4f)  /* 拒绝 ADC 节拍级霍尔毛刺 (s) */
#define MOTOR_HALL_INTERP_START_RPM (150.0f)   /* 以下保持扇区中心，保证低速启动转矩 */
#define MOTOR_HALL_INTERP_FULL_RPM  (300.0f)   /* 以上使用完整扇区内角度预测 */
/* Private typedef -----------------------------------------------------------*/
/**
 * @description: 霍尔角度运行时（默认单实例 + *State 可测）
 */
typedef struct {
    int last_state;       /* 上一霍尔状态 0..7；-1 表示尚无 */
    bool have_state;      /* 是否已采过有效扇区 */
    float theta_rad;      /* 最近有效电角 (rad) */
    float mech_rpm;       /* 多边沿周期滤波机械转速 (RPM)，显示用 */
    float mech_rpm_raw;   /* 最近单边沿周期机械转速 (RPM) */
    float mech_rpm_fast;  /* Hall-PLL 机械转速，供速度环使用 (RPM) */
    bool angle_valid;     /* 本拍电角是否可信 */
    bool speed_valid;     /* 本拍转速是否可信 */
    bool table_ready;     /* 本实例已完成霍尔表结构校验 */
    float sector_timer_s; /* 距上次换向时间 (s) */
    float last_period_s;  /* 最近合法边沿周期 (s) */
    float period_sum_s;   /* 滤波窗口周期总和 (s) */
    float periods_s[MOTOR_HALL_SPEED_WINDOW];
    uint8_t period_count;
    uint8_t period_index;
    int8_t direction;     /* +1 正转，-1 反转，0 未知 */
    int8_t pll_count;     /* PLL 电扇区计数 0..5 */
    float pll_pos;        /* PLL 位置 (电扇区 counts) */
    float pll_vel;        /* PLL 速度 (电扇区 counts/s) */
    bool pll_low_speed_active;
} motor_hall_angle_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 判断霍尔表是否可用于运行（恰 6 个有效扇区、2 个无效）
 * @param {const int8_t} *table  长度 MOTOR_HALL_TABLE_SIZE
 * @return {bool} true=表就绪
 * @author: lkc
 */
bool Motor_Hall_TableReady(const int8_t *table);

/**
 * @description: 由霍尔状态查表得到电角度
 * @param {const int8_t} *table  霍尔表
 * @param {int} hall_state       霍尔状态 0..7
 * @param {float} *theta_rad     输出电角 (rad)
 * @return {bool} true=该扇区有效；false=越界/非法表项
 * @author: lkc
 */
bool Motor_Hall_DecodeAngle(const int8_t *table, int hall_state, float *theta_rad);

/**
 * @description: 复位霍尔角度运行时
 * @param {motor_hall_angle_t} *st  状态
 * @return {*}
 * @author: lkc
 */
void Motor_Hall_AngleReset(motor_hall_angle_t *st);

/**
 * @description: 用本拍霍尔状态更新电角与转速估计
 * @param {motor_hall_angle_t} *st  状态
 * @param {const int8_t} *table     霍尔表
 * @param {int} hall_state          当前霍尔 0..7
 * @param {float} pole_pairs        极对数（电转速→机械转速）
 * @param {float} dt_s              本拍时间 (s)
 * @return {*}
 * @author: lkc
 */
void Motor_Hall_AngleUpdate(motor_hall_angle_t *st, const int8_t *table, int hall_state, float pole_pairs,
                            float dt_s);

/**
 * @description: 默认实例复位
 * @return {*}
 * @author: lkc
 */
void Motor_Hall_AngleResetDefault(void);

/**
 * @description: 默认实例更新
 * @param {const int8_t} *table  霍尔表
 * @param {int} hall_state       当前霍尔
 * @param {float} pole_pairs     极对数
 * @param {float} dt_s           本拍时间 (s)
 * @return {*}
 * @author: lkc
 */
void Motor_Hall_AngleUpdateDefault(const int8_t *table, int hall_state, float pole_pairs, float dt_s);

/**
 * @description: 默认实例电角 (rad)；angle_valid=false 时值勿单独使用
 * @return {float} 电角度 (rad)
 * @author: lkc
 */
float Motor_Hall_AngleGetTheta(void);

/**
 * @description: 默认实例机械转速 (RPM)
 * @return {float} 机械转速 (RPM)
 * @author: lkc
 */
float Motor_Hall_AngleGetMechRpm(void);

/** @description: 默认实例最近单边沿原始机械转速 (RPM) */
float Motor_Hall_AngleGetRawMechRpm(void);

/** @description: 默认实例 Hall-PLL 机械转速，供控制环使用 (RPM) */
float Motor_Hall_AngleGetFastMechRpm(void);

/** @description: 默认实例距最近霍尔边沿的时间 (s) */
float Motor_Hall_AngleGetEdgeAge(void);

/** @description: 默认实例当前霍尔状态；尚无有效状态返回 -1 */
int Motor_Hall_AngleGetState(void);

/**
 * @description: 默认实例电角是否有效
 * @return {bool} true=电角可信
 * @author: lkc
 */
bool Motor_Hall_AngleIsAngleValid(void);

/**
 * @description: 默认实例转速是否有效
 * @return {bool} true=转速可信
 * @author: lkc
 */
bool Motor_Hall_AngleIsSpeedValid(void);
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
