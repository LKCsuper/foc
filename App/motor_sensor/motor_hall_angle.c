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
static void Motor_Hall_InterpolateTheta(motor_hall_angle_t *st, float sector_center, float pole_pairs);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 在当前 60° 霍尔扇区内按滤波速度推进电角度
 * @detail: 标定表保存扇区中心；进入扇区时从中心前 30° 起步，并限制不越过后边界。
 */
static void Motor_Hall_InterpolateTheta(motor_hall_angle_t *st, float sector_center, float pole_pairs)
{
    float progress;
    float blend;
    float delta;
    float theta;
    float rpm_abs;

    if (st == NULL || st->direction == 0 || !st->speed_valid || fabsf(st->mech_rpm_fast) < 1.0f) {
        return;
    }

    rpm_abs = fabsf(st->mech_rpm_fast);
    if (rpm_abs <= MOTOR_HALL_INTERP_START_RPM) {
        return;
    }
    blend = (rpm_abs - MOTOR_HALL_INTERP_START_RPM) /
            (MOTOR_HALL_INTERP_FULL_RPM - MOTOR_HALL_INTERP_START_RPM);
    if (blend > 1.0f) {
        blend = 1.0f;
    }

    progress = rpm_abs * pole_pairs * (2.0f * (float)M_PI / 60.0f) * st->sector_timer_s;
    if (progress > (float)M_PI / 3.0f) {
        progress = (float)M_PI / 3.0f;
    }
    delta = (float)st->direction * (progress - ((float)M_PI / 6.0f));
    theta = sector_center + blend * delta;
    utils_norm_angle_rad(&theta);
    st->theta_rad = theta;
}

/**
 * @description: 判断霍尔表是否可用于运行（恰 6 个有效扇区、2 个无效）
 * @param {const int8_t} *table  长度 MOTOR_HALL_TABLE_SIZE
 * @return {bool} true=表就绪
 * @author: lkc
 */
bool Motor_Hall_TableReady(const int8_t *table)
{
    int i;
    int j;
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
    if ((valid != 6) || (invalid != 2)) {
        return false;
    }

    /*
     * 六个有效扇区不能是同一个（或几乎同一个）电角。
     * 标准间距约为 200/6=33；小于 10（18°）说明标定单位/采样异常。
     */
    for (i = 0; i < (int)MOTOR_HALL_TABLE_SIZE; i++) {
        uint8_t a = (uint8_t)table[i];

        if (a == 255U) {
            continue;
        }
        for (j = i + 1; j < (int)MOTOR_HALL_TABLE_SIZE; j++) {
            uint8_t b = (uint8_t)table[j];
            int distance;

            if (b == 255U) {
                continue;
            }
            distance = (int)a - (int)b;
            if (distance < 0) {
                distance = -distance;
            }
            if (distance > 100) {
                distance = 200 - distance;
            }
            if (distance < 10) {
                return false;
            }
        }
    }
    return true;
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
    int i;

    if (st == NULL) {
        return;
    }
    st->last_state = -1;
    st->have_state = false;
    st->theta_rad = 0.0f;
    st->mech_rpm = 0.0f;
    st->mech_rpm_raw = 0.0f;
    st->mech_rpm_fast = 0.0f;
    st->angle_valid = false;
    st->speed_valid = false;
    st->table_ready = false;
    st->sector_timer_s = 0.0f;
    st->last_period_s = 0.0f;
    st->period_sum_s = 0.0f;
    st->period_count = 0U;
    st->period_index = 0U;
    st->direction = 0;
    for (i = 0; i < (int)MOTOR_HALL_SPEED_WINDOW; i++) {
        st->periods_s[i] = 0.0f;
    }
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
    if (!st->table_ready) {
        st->table_ready = Motor_Hall_TableReady(table);
    }
    if (!st->table_ready || !Motor_Hall_DecodeAngle(table, hall_state, &theta)) {
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
            float abs_d_theta;
            float period_s = st->sector_timer_s;
            float calibrated_rpm;
            int8_t direction;

            d_theta = theta - theta_prev;
            while (d_theta > (float)M_PI) {
                d_theta -= 2.0f * (float)M_PI;
            }
            while (d_theta < -(float)M_PI) {
                d_theta += 2.0f * (float)M_PI;
            }
            abs_d_theta = fabsf(d_theta);
            /* 合法相邻霍尔边沿约跨 60°；拒绝毛刺、重复码和跨扇区跳码。 */
            if (abs_d_theta >= ((float)M_PI / 6.0f) && abs_d_theta <= ((float)M_PI / 2.0f)) {
                direction = d_theta > 0.0f ? 1 : -1;
                if (st->direction != 0 && direction != st->direction) {
                    st->period_sum_s = 0.0f;
                    st->period_count = 0U;
                    st->period_index = 0U;
                }
                st->direction = direction;

                /* 每个合法边沿固定跨 1/6 电角周，避免标定表角间距误差污染测速。 */
                st->mech_rpm_raw = (float)direction * 10.0f / (pp * period_s);
                /*
                 * 快速支路按标定表的实际扇区角差换算，避免不等宽扇区造成单边沿周期偏差；
                 * 低通只承担抗抖，六边沿支路仍保留给显示和稳态审核。
                 */
                calibrated_rpm = (d_theta / (2.0f * (float)M_PI)) * 60.0f / (pp * period_s);
                if (fabsf(st->mech_rpm_fast) < 1.0f || st->period_count == 0U) {
                    st->mech_rpm_fast = calibrated_rpm;
                } else {
                    st->mech_rpm_fast += MOTOR_HALL_FAST_ALPHA * (calibrated_rpm - st->mech_rpm_fast);
                }
                st->last_period_s = period_s;
                if (st->period_count == MOTOR_HALL_SPEED_WINDOW) {
                    st->period_sum_s -= st->periods_s[st->period_index];
                } else {
                    st->period_count++;
                }
                st->periods_s[st->period_index] = period_s;
                st->period_sum_s += period_s;
                st->period_index = (uint8_t)((st->period_index + 1U) % MOTOR_HALL_SPEED_WINDOW);
                elec_rps = ((float)st->period_count / 6.0f) / st->period_sum_s;
                st->mech_rpm = (float)direction * (elec_rps * 60.0f) / pp;
                st->speed_valid = true;
            }
        }
        st->last_state = hall_state;
        st->sector_timer_s = 0.0f;
        Motor_Hall_InterpolateTheta(st, theta, pp);
        return;
    }

    /* 同扇区：超时无换向 → 停转，转速有效为 0 */
    if (st->sector_timer_s >= MOTOR_HALL_ANGLE_STALL_S) {
        st->mech_rpm = 0.0f;
        st->mech_rpm_raw = 0.0f;
        st->mech_rpm_fast = 0.0f;
        st->speed_valid = true;
        st->period_sum_s = 0.0f;
        st->period_count = 0U;
        st->period_index = 0U;
        st->direction = 0;
    } else if (st->direction != 0 && st->last_period_s > 1.0e-6f &&
               st->sector_timer_s > (1.5f * st->last_period_s)) {
        /* 未等到预期边沿时，按当前边沿年龄限制可能速度，避免 PI 长时间使用旧高速。 */
        float age_limit_rpm = 10.0f / (pp * st->sector_timer_s);
        if (fabsf(st->mech_rpm_fast) > age_limit_rpm) {
            st->mech_rpm_fast = (float)st->direction * age_limit_rpm;
        }
    }
    Motor_Hall_InterpolateTheta(st, theta, pp);
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

float Motor_Hall_AngleGetRawMechRpm(void)
{
    return s_hall_angle.mech_rpm_raw;
}

float Motor_Hall_AngleGetFastMechRpm(void)
{
    return s_hall_angle.mech_rpm_fast;
}

float Motor_Hall_AngleGetEdgeAge(void)
{
    return s_hall_angle.sector_timer_s;
}

int Motor_Hall_AngleGetState(void)
{
    return s_hall_angle.have_state ? s_hall_angle.last_state : -1;
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
