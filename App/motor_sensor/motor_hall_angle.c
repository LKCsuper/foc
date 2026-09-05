/*
 * @Description: 霍尔表正常解码（扇区→电角、转速与非法状态）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include <math.h>
#include "motor_hall_angle.h"
#include "motor_math.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static motor_hall_angle_t s_hall_angle;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 判断霍尔表是否可用于运行（恰 6 个有效扇区、2 个无效）
 * @param {const int8_t} *table  长度 MOTOR_HALL_TABLE_SIZE
 * @return {bool} true=表就绪
 * @author: lkc
 */
bool Motor_Hall_TableReady(const int8_t *table)
{
    int i;
    int valid = 0;
    int invalid = 0;

    if (table == NULL) {
        return false;
    }
    for (i = 0; i < (int)MOTOR_HALL_TABLE_SIZE; i++) {
        uint8_t raw = (uint8_t)table[i];

        /* int8 存 128..200 时为负，按无符号字节解释；255/-1 为无效 */
        if (raw == 255U) {
            invalid++;
        } else if (raw <= 200U) {
            valid++;
        } else {
            return false;
        }
    }
    return (valid == 6) && (invalid == 2);
}

/**
 * @description: 由霍尔状态查表得到电角度
 * @param {const int8_t} *table  霍尔表
 * @param {int} hall_state       霍尔状态 0..7
 * @param {float} *theta_rad     输出电角 (rad)
 * @return {bool} true=该扇区有效；false=越界/非法表项
 * @author: lkc
 */
bool Motor_Hall_DecodeAngle(const int8_t *table, int hall_state, float *theta_rad)
{
    uint8_t raw;

    if (table == NULL || theta_rad == NULL || hall_state < 0 || hall_state > 7) {
        return false;
    }
    raw = (uint8_t)table[hall_state];
    if (raw == 255U || raw > 200U) {
        return false;
    }
    /* raw = deg * 200 / 360 → rad = raw / 200 * 2π */
    *theta_rad = ((float)raw / 200.0f) * 2.0f * (float)M_PI;
    utils_norm_angle_rad(theta_rad);
    return true;
}

/**
 * @description: 复位霍尔角度运行时
 * @param {motor_hall_angle_t} *st  状态
 * @return {*}
 * @author: lkc
 */
void Motor_Hall_AngleReset(motor_hall_angle_t *st)
{
    if (st == NULL) {
        return;
    }
    st->last_state = -1;
    st->have_state = false;
    st->theta_rad = 0.0f;
    st->mech_rpm = 0.0f;
    st->angle_valid = false;
    st->speed_valid = false;
    st->sector_timer_s = 0.0f;
}

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
                            float dt_s)
{
    float theta;
    float pp = pole_pairs;
    float d_theta;
    float elec_rps;

    if (st == NULL) {
        return;
    }
    if (dt_s < 0.0f) {
        dt_s = 0.0f;
    }
    if (pp < 1.0f) {
        pp = 1.0f;
    }

    st->angle_valid = false;
    if (!Motor_Hall_TableReady(table) || !Motor_Hall_DecodeAngle(table, hall_state, &theta)) {
        st->speed_valid = false;
        st->mech_rpm = 0.0f;
        return;
    }

    st->theta_rad = theta;
    st->angle_valid = true;
    st->sector_timer_s += dt_s;

    if (!st->have_state) {
        st->last_state = hall_state;
        st->have_state = true;
        st->mech_rpm = 0.0f;
        st->speed_valid = false;
        st->sector_timer_s = 0.0f;
        return;
    }

    if (hall_state != st->last_state) {
        float theta_prev;

        if (st->sector_timer_s > 1.0e-6f && Motor_Hall_DecodeAngle(table, st->last_state, &theta_prev)) {
            d_theta = theta - theta_prev;
            while (d_theta > (float)M_PI) {
                d_theta -= 2.0f * (float)M_PI;
            }
            while (d_theta < -(float)M_PI) {
                d_theta += 2.0f * (float)M_PI;
            }
            elec_rps = (d_theta / (2.0f * (float)M_PI)) / st->sector_timer_s;
            st->mech_rpm = (elec_rps * 60.0f) / pp;
            st->speed_valid = true;
        }
        st->last_state = hall_state;
        st->sector_timer_s = 0.0f;
        return;
    }

    /* 同扇区：超时无换向 → 停转，转速有效为 0 */
    if (st->sector_timer_s >= MOTOR_HALL_ANGLE_STALL_S) {
        st->mech_rpm = 0.0f;
        st->speed_valid = true;
    }
}

/**
 * @description: 默认实例复位
 * @return {*}
 * @author: lkc
 */
void Motor_Hall_AngleResetDefault(void)
{
    Motor_Hall_AngleReset(&s_hall_angle);
}

/**
 * @description: 默认实例更新
 * @param {const int8_t} *table  霍尔表
 * @param {int} hall_state       当前霍尔
 * @param {float} pole_pairs     极对数
 * @param {float} dt_s           本拍时间 (s)
 * @return {*}
 * @author: lkc
 */
void Motor_Hall_AngleUpdateDefault(const int8_t *table, int hall_state, float pole_pairs, float dt_s)
{
    Motor_Hall_AngleUpdate(&s_hall_angle, table, hall_state, pole_pairs, dt_s);
}

/**
 * @description: 默认实例电角 (rad)；angle_valid=false 时值勿单独使用
 * @return {float} 电角度 (rad)
 * @author: lkc
 */
float Motor_Hall_AngleGetTheta(void)
{
    return s_hall_angle.theta_rad;
}

/**
 * @description: 默认实例机械转速 (RPM)
 * @return {float} 机械转速 (RPM)
 * @author: lkc
 */
float Motor_Hall_AngleGetMechRpm(void)
{
    return s_hall_angle.mech_rpm;
}

/**
 * @description: 默认实例电角是否有效
 * @return {bool} true=电角可信
 * @author: lkc
 */
bool Motor_Hall_AngleIsAngleValid(void)
{
    return s_hall_angle.angle_valid;
}

/**
 * @description: 默认实例转速是否有效
 * @return {bool} true=转速可信
 * @author: lkc
 */
bool Motor_Hall_AngleIsSpeedValid(void)
{
    return s_hall_angle.speed_valid;
}

#ifdef __cplusplus
}
#endif
