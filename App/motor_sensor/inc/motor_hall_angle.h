/*
 * @Description: 霍尔表正常解码（扇区→电角、转速与非法状态）
 * @Author: lkc
 *
 * 学习要点：
 *   1) hall_table[i]：0..200 = 电角编码（deg×200/360），-1 无效（0/7 常为无效）。
 *   2) 表未就绪或当前扇区非法 → angle_valid=false，勿当有效零速闭环。
 *   3) 转速由扇区切换估计；超时无切换视为停转（rpm=0 且 speed_valid）。
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
#define MOTOR_HALL_ANGLE_STALL_S (0.2f) /* 无换向超过该时间视为停转 (s) */
#define MOTOR_HALL_SPEED_WINDOW  (6U)    /* 最近边沿周期窗口 */
#define MOTOR_HALL_FAST_ALPHA    (0.35f) /* 单边沿标定速度低通系数 */
#define MOTOR_HALL_INTERP_START_RPM (150.0f) /* 以下保持扇区中心，保证低速启动转矩 */
#define MOTOR_HALL_INTERP_FULL_RPM  (300.0f) /* 以上使用完整扇区内角度预测 */
/* Private typedef -----------------------------------------------------------*/
/**
 * @description: 霍尔角度运行时（默认单实例 + *State 可测）
 */
typedef struct {
    int last_state;       /* 上一霍尔状态 0..7；-1 表示尚无 */
    bool have_state;      /* 是否已采过有效扇区 */
    float theta_rad;      /* 最近有效电角 (rad) */
    float mech_rpm;       /* 多边沿周期滤波机械转速 (RPM) */
    float mech_rpm_raw;   /* 最近单边沿周期机械转速 (RPM) */
    float mech_rpm_fast;  /* 标定扇区角差快速速度，供速度环使用 (RPM) */
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

/** @description: 默认实例快速机械转速，供控制环使用 (RPM) */
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
