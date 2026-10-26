/*
 * @Description: 霍尔表正常解码（扇区→电角、转速与非法状态）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include "common_assert.h"
#include <math.h>
#include "motor_hall_angle.h"
#include "motor_math.h"
#if defined(__CC_ARM)
/* 位于 ADC 中断控制路径，与其他实时模块一致局部 O2；不启用快速浮点，公式不变。 */
#pragma O2
#endif
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static motor_hall_angle_t s_hall_angle;
/* Private function prototypes -----------------------------------------------*/
static void Motor_Hall_InterpolateTheta(motor_hall_angle_t *st, const int8_t *table, float sector_center,
                                        float pole_pairs);
static void Motor_Hall_PllReset(motor_hall_angle_t *st);
static void Motor_Hall_PllRun(motor_hall_angle_t *st, float pole_pairs, float dt_s);
static void Motor_Hall_UpdateSpeedEstimate(motor_hall_angle_t *st, float pole_pairs, float dt_s);
static float Motor_Hall_WrapCpr(float x);
static float Motor_Hall_WrapDelta(float x);
static float Motor_Hall_WrapPi(float x);
static bool Motor_Hall_NeighborHalfWidths(const int8_t *table, float sector_center, float *half_neg,
                                          float *half_pos);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 把电角折到 (−π, π]。
 * @detail: 非有限值或绝对值过大时返回 0，避免在中断里死循环。
 * @param {float} x  电角 (rad)
 * @return {float} 折回后的电角 (rad)
 * @author: lkc
 */
static float Motor_Hall_WrapPi(float x)
{
    if (!(x > -1.0e6f) || !(x < 1.0e6f)) return 0.0f;
    while (x > (float)M_PI) {
        x -= 2.0f * (float)M_PI;
    }
    while (x < -(float)M_PI) {
        x += 2.0f * (float)M_PI;
    }
    return x;
}

/**
 * @description: 取当前扇区中心到相邻有效表项的半宽
 * @param {const int8_t} *table  霍尔表
 * @param {float} sector_center  当前扇区中心 (rad)
 * @param {float} *half_neg      到更小电角邻居的半宽 (rad)
 * @param {float} *half_pos      到更大电角邻居的半宽 (rad)
 * @return {bool} true=找到两侧邻居
 */
static bool Motor_Hall_NeighborHalfWidths(const int8_t *table, float sector_center, float *half_neg,
                                          float *half_pos)
{
    float min_pos = 4.0f;
    float min_neg = -4.0f;
    bool have_pos = false;
    bool have_neg = false;
    int i;

    if (table == NULL || half_neg == NULL || half_pos == NULL) return false;
    for (i = 0; i < (int)MOTOR_HALL_TABLE_SIZE; i++) {
        float other;
        float delta;

        if (!Motor_Hall_DecodeAngle(table, i, &other)) {
            continue;
        }
        delta = Motor_Hall_WrapPi(other - sector_center);
        if (delta > 1.0e-3f && delta < min_pos) {
            min_pos = delta;
            have_pos = true;
        } else if (delta < -1.0e-3f && delta > min_neg) {
            min_neg = delta;
            have_neg = true;
        }
    }
    if (!have_pos || !have_neg) return false;
    *half_pos = 0.5f * min_pos;
    *half_neg = 0.5f * (-min_neg);
    return true;
}

/**
 * @description: 在当前霍尔扇区内按滤波速度推进电角度
 * @detail: 标定表保存扇区中心。进入扇区时从与上一表项的实际边界起步，并限制不超过下一表项边界。
 *          低于 MOTOR_HALL_INTERP_START_RPM 保持中心，避免低速 60° 锁偏。
 * @param {motor_hall_angle_t *} st 实例状态，不可为空
 * @param {const int8_t *} table 八项霍尔电角编码表
 * @param {float} sector_center 当前扇区中心电角度 (rad)
 * @param {float} pole_pairs 极对数
 * @return {void} 无
 */
static void Motor_Hall_InterpolateTheta(motor_hall_angle_t *st, const int8_t *table, float sector_center,
                                        float pole_pairs)
{
    float progress;
    float blend;
    float delta;
    float theta;
    float rpm_abs;
    float half_neg = (float)M_PI / 6.0f;
    float half_pos = (float)M_PI / 6.0f;
    float half_back;
    float half_fwd;

    COMMON_ASSERT(st != NULL);
    if (st->direction == 0 || !st->speed_valid || fabsf(st->mech_rpm_fast) < 1.0f) return;

    rpm_abs = fabsf(st->mech_rpm_fast);
    if (rpm_abs <= MOTOR_HALL_INTERP_START_RPM) return;
    blend = (rpm_abs - MOTOR_HALL_INTERP_START_RPM) /
            (MOTOR_HALL_INTERP_FULL_RPM - MOTOR_HALL_INTERP_START_RPM);
    if (blend > 1.0f) {
        blend = 1.0f;
    }
    (void)Motor_Hall_NeighborHalfWidths(table, sector_center, &half_neg, &half_pos);
    if (st->direction > 0) {
        half_back = half_neg;
        half_fwd = half_pos;
    } else {
        half_back = half_pos;
        half_fwd = half_neg;
    }

    progress = rpm_abs * pole_pairs * (2.0f * (float)M_PI / 60.0f) * st->sector_timer_s;
    if (progress > (half_back + half_fwd)) {
        progress = half_back + half_fwd;
    }
    delta = (float)st->direction * (progress - half_back);
    theta = sector_center + blend * delta;
    utils_norm_angle_rad(&theta);
    st->theta_rad = theta;
}

/**
 * @description: 把 PLL 位置折回 [0, cpr)。
 * @detail: 非有限值或绝对值过大时返回 0，避免在中断里死循环。
 * @param {float} x  位置 (count)
 * @return {float} 折回后的位置 (count)
 * @author: lkc
 */
static float Motor_Hall_WrapCpr(float x)
{
    float n;

    /* Inf/NaN 或超大值不能用 while 回绕：30 kHz ISR 里会卡死，UART 随之饿死。 */
    if (!(x > -1.0e6f) || !(x < 1.0e6f)) return 0.0f;
    n = (float)(int)(x / MOTOR_HALL_PLL_CPR);
    x -= MOTOR_HALL_PLL_CPR * n;
    if (x < 0.0f) {
        x += MOTOR_HALL_PLL_CPR;
    }
    if (x >= MOTOR_HALL_PLL_CPR) {
        x -= MOTOR_HALL_PLL_CPR;
    }
    return x;
}

/**
 * @description: 把 PLL 位置误差折到 ±cpr/2。
 * @param {float} x  未折回的误差 (count)
 * @return {float} 最短路径误差 (count)；非法输入返回 0
 * @author: lkc
 */
static float Motor_Hall_WrapDelta(float x)
{
    const float half = 0.5f * MOTOR_HALL_PLL_CPR;
    float n;

    if (!(x > -1.0e6f) || !(x < 1.0e6f)) return 0.0f;
    n = (float)(int)(x / MOTOR_HALL_PLL_CPR);
    x -= MOTOR_HALL_PLL_CPR * n;
    if (x > half) {
        x -= MOTOR_HALL_PLL_CPR;
    } else if (x <= -half) {
        x += MOTOR_HALL_PLL_CPR;
    }
    return x;
}

/**
 * @description: 清零霍尔 PLL 位置和速度，并进入低速模式。
 * @param {motor_hall_angle_t *} st  霍尔角度状态，调用方保证非空
 * @return {void} 无
 * @author: lkc
 */
static void Motor_Hall_PllReset(motor_hall_angle_t *st)
{
    st->pll_count = 0;
    st->pll_pos = 0.0f;
    st->pll_vel = 0.0f;
    st->mech_rpm_fast = 0.0f;
    st->pll_low_speed_active = true;
}

/**
 * @description: ODrive 式临界阻尼 PLL，把霍尔扇区计数变成连续机械转速
 * @detail: kp=2·bw，ki=0.25·kp²。Hall cpr=6，bw 远低于编码器默认 1000，否则 72 RPM 会打齿。
 *          时基异常（dt·kp≥1）时钉在扇区中心并清速度。
 * @param {motor_hall_angle_t *} st 实例状态
 * @param {float} pole_pairs 极对数
 * @param {float} dt_s 本次推进时间间隔 (s)
 * @return {void} 无
 */
static void Motor_Hall_PllRun(motor_hall_angle_t *st, float pole_pairs, float dt_s)
{
    const float kp = 2.0f * MOTOR_HALL_PLL_BW_HZ;
    const float ki = 0.25f * kp * kp;
    float delta_pos;
    float snap;

    if (dt_s <= 0.0f || (dt_s * kp) >= 1.0f) {
        st->pll_vel = 0.0f;
        st->pll_pos = (float)st->pll_count + 0.5f;
        st->mech_rpm_fast = 0.0f;
        return;
    }

    st->pll_pos += dt_s * st->pll_vel;
    st->pll_pos = Motor_Hall_WrapCpr(st->pll_pos);
    /* 必须比较连续估计位置；取整会令同扇区误差恒定，PLL 无法正确估速。 */
    delta_pos = Motor_Hall_WrapDelta((float)st->pll_count - st->pll_pos);
    st->pll_pos += dt_s * kp * delta_pos;
    st->pll_pos = Motor_Hall_WrapCpr(st->pll_pos);
    st->pll_vel += dt_s * ki * delta_pos;
    if (st->pll_vel > MOTOR_HALL_PLL_VEL_MAX) {
        st->pll_vel = MOTOR_HALL_PLL_VEL_MAX;
    } else if (st->pll_vel < -MOTOR_HALL_PLL_VEL_MAX) {
        st->pll_vel = -MOTOR_HALL_PLL_VEL_MAX;
    }
    snap = 0.5f * dt_s * ki;
    if (fabsf(st->pll_vel) < snap) {
        st->pll_vel = 0.0f;
    }
    st->mech_rpm_fast = st->pll_vel * (60.0f / MOTOR_HALL_PLL_CPR) / pole_pairs;
    if (st->mech_rpm_fast > 1.0f) {
        st->direction = 1;
    } else if (st->mech_rpm_fast < -1.0f) {
        st->direction = -1;
    }
}

/**
 * @description: 低速用 PLL 估速，转速升高后改用边沿间隔转速。
 * @param {motor_hall_angle_t *} st  霍尔角度状态
 * @param {float} pole_pairs        极对数
 * @param {float} dt_s              距上一拍时间 (s)
 * @return {void} 无
 * @author: lkc
 */
static void Motor_Hall_UpdateSpeedEstimate(motor_hall_angle_t *st, float pole_pairs, float dt_s)
{
    float rpm_abs = fabsf(st->mech_rpm);

    /* Hall 六扇区 PLL 带宽适合低速；高速直接起转可能相位跨半周而误锁。 */
    if (st->pll_low_speed_active && rpm_abs >= MOTOR_HALL_PLL_EXIT_RPM) {
        st->pll_low_speed_active = false;
    } else if (!st->pll_low_speed_active && rpm_abs <= MOTOR_HALL_PLL_ENTER_RPM) {
        st->pll_low_speed_active = true;
        st->pll_pos = (float)st->pll_count;
        st->pll_vel = st->mech_rpm * pole_pairs * (MOTOR_HALL_PLL_CPR / 60.0f);
    }

    if (st->pll_low_speed_active) {
        Motor_Hall_PllRun(st, pole_pairs, dt_s);
    } else {
        st->mech_rpm_fast = st->mech_rpm;
    }
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

    if (table == NULL) return false;
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
    if ((valid != 6) || (invalid != 2)) return false;

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
            if (distance < 10) return false;
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

    if (table == NULL || theta_rad == NULL || hall_state < 0 || hall_state > 7) return false;
    raw = (uint8_t)table[hall_state];
    if (raw == 255U || raw > 200U) return false;
    /* raw = deg * 200 / 360 → rad = raw / 200 * 2π */
    *theta_rad = ((float)raw / 200.0f) * 2.0f * (float)M_PI;
    utils_norm_angle_rad(theta_rad);
    return true;
}

/**
 * @description: 复位霍尔角度运行时
 * @param {motor_hall_angle_t} *st  状态
 * @return {void} 无
 * @author: lkc
 */
void Motor_Hall_AngleReset(motor_hall_angle_t *st)
{
    int i;

    COMMON_ASSERT(st != NULL);
    st->last_state = -1;
    st->have_state = false;
    st->theta_rad = 0.0f;
    st->mech_rpm = 0.0f;
    st->mech_rpm_raw = 0.0f;
    st->angle_valid = false;
    st->speed_valid = false;
    st->table_ready = false;
    st->sector_timer_s = 0.0f;
    st->last_period_s = 0.0f;
    st->period_sum_s = 0.0f;
    st->period_count = 0U;
    st->period_index = 0U;
    st->direction = 0;
    Motor_Hall_PllReset(st);
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
 * @return {void} 无
 * @author: lkc
 */
void Motor_Hall_AngleUpdate(motor_hall_angle_t *st, const int8_t *table, int hall_state, float pole_pairs,
                            float dt_s)
{
    float theta;
    float pp = pole_pairs;
    float d_theta;
    float elec_rps;

    COMMON_ASSERT(st != NULL);
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
        st->speed_valid = true;
        st->sector_timer_s = 0.0f;
        Motor_Hall_PllReset(st);
        return;
    }

    if (hall_state != st->last_state) {
        float theta_prev;

        if (st->sector_timer_s > 1.0e-6f && Motor_Hall_DecodeAngle(table, st->last_state, &theta_prev)) {
            float abs_d_theta;
            float period_s = st->sector_timer_s;
            int8_t direction;

            d_theta = theta - theta_prev;
            while (d_theta > (float)M_PI) {
                d_theta -= 2.0f * (float)M_PI;
            }
            while (d_theta < -(float)M_PI) {
                d_theta += 2.0f * (float)M_PI;
            }
            abs_d_theta = fabsf(d_theta);
            /* 合法相邻霍尔边沿约跨 60°；过短周期视为 ADC 节拍毛刺，避免 PLL 计数狂奔。 */
            if ((period_s >= MOTOR_HALL_MIN_PERIOD_S) && (abs_d_theta >= ((float)M_PI / 6.0f)) &&
                (abs_d_theta <= ((float)M_PI / 2.0f))) {
                direction = d_theta > 0.0f ? 1 : -1;
                if (st->direction != 0 && direction != st->direction) {
                    st->period_sum_s = 0.0f;
                    st->period_count = 0U;
                    st->period_index = 0U;
                }
                st->direction = direction;
                st->pll_count = (int8_t)(st->pll_count + direction);
                if (st->pll_count >= 6) {
                    st->pll_count = (int8_t)(st->pll_count - 6);
                } else if (st->pll_count < 0) {
                    st->pll_count = (int8_t)(st->pll_count + 6);
                }

                /* 每个合法边沿固定跨 1/6 电角周，避免标定表角间距误差污染显示测速。 */
                st->mech_rpm_raw = (float)direction * 10.0f / (pp * period_s);
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
        Motor_Hall_UpdateSpeedEstimate(st, pp, dt_s);
        Motor_Hall_InterpolateTheta(st, table, theta, pp);
        return;
    }

    /* 同扇区：超时无换向 → 停转，转速有效为 0 */
    if (st->sector_timer_s >= MOTOR_HALL_ANGLE_STALL_S) {
        st->mech_rpm = 0.0f;
        st->mech_rpm_raw = 0.0f;
        st->speed_valid = true;
        st->period_sum_s = 0.0f;
        st->period_count = 0U;
        st->period_index = 0U;
        st->direction = 0;
        Motor_Hall_PllReset(st);
    } else {
        Motor_Hall_UpdateSpeedEstimate(st, pp, dt_s);
    }
    Motor_Hall_InterpolateTheta(st, table, theta, pp);
}

/**
 * @description: 默认实例复位
 * @return {void} 无
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
 * @return {void} 无
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
 * @description: 读取默认实例最近单边沿原始机械转速。
 * @return {float} 机械转速 (RPM)
 * @author: lkc
 */
float Motor_Hall_AngleGetRawMechRpm(void)
{
    return s_hall_angle.mech_rpm_raw;
}

/**
 * @description: 读取默认实例 Hall-PLL 机械转速，供控制环使用。
 * @return {float} 机械转速 (RPM)
 * @author: lkc
 */
float Motor_Hall_AngleGetFastMechRpm(void)
{
    return s_hall_angle.mech_rpm_fast;
}

/**
 * @description: 读取默认实例距最近一次霍尔边沿的时间。
 * @return {float} 边沿年龄 (s)
 * @author: lkc
 */
float Motor_Hall_AngleGetEdgeAge(void)
{
    return s_hall_angle.sector_timer_s;
}

/**
 * @description: 读取默认实例最近一次有效霍尔状态。
 * @return {int} 霍尔状态 1..6；尚未收到有效状态时返回 -1
 * @author: lkc
 */
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
