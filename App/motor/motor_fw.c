/*
 * @Description: 弱磁：电压利用率触发负 Id
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "motor_fw.h"
#include "common_assert.h"
#include <math.h>
#include <stddef.h>
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 复位弱磁状态
 * @param {motor_fw_t} *fw  弱磁实例
 * @return {*}
 * @author: lkc
 */
void Motor_Fw_Reset(motor_fw_t *fw)
{
    COMMON_ASSERT(fw != NULL);
    fw->id = 0.0f;
    fw->last_vd = 0.0f;
    fw->last_vq = 0.0f;
}

/**
 * @description: 电流圆内优先保留 Iq 后的弱磁 |Id| 余量
 * @param {float} iq  Iq 目标 (A)
 * @param {float} i_budget  电流圆半径 (A)
 * @return {float} |Id| 余量 (A)
 * @author: lkc
 */
float Motor_Fw_IdRoom(float iq, float i_budget)
{
    float iq_abs;

    if (i_budget <= 0.0f) return 0.0f;
    iq_abs = fabsf(iq);
    if (iq_abs >= i_budget) return 0.0f;
    return sqrtf(i_budget * i_budget - iq_abs * iq_abs);
}

/**
 * @description: 按电压利用率斜坡给出负 Id
 * @param {motor_fw_t} *fw  弱磁实例
 * @param {float} current_max  |Id| 上限 (A)
 * @param {float} duty_start  进入门槛
 * @param {float} ramp_a_s  斜坡 (A/s)
 * @param {float} vd  D 轴电压 (V)
 * @param {float} vq  Q 轴电压 (V)
 * @param {float} v_limit  电压上限 (V)
 * @param {float} dt  控制周期 (s)
 * @return {float} Id_fw (A)
 * @author: lkc
 */
float Motor_Fw_Step(motor_fw_t *fw, float current_max, float duty_start, float ramp_a_s, float vd, float vq,
                    float v_limit, float dt)
{
    float mod;
    float duty_exit;
    float target;
    float step;
    float id;

    if (fw == NULL) return 0.0f;
    if (current_max <= 0.0f || v_limit <= 0.0f) {
        fw->id = 0.0f;
        return 0.0f;
    }

    mod = sqrtf(vd * vd + vq * vq) / v_limit;
    if (duty_start < 0.50f) {
        duty_start = 0.50f;
    }
    if (duty_start > 0.99f) {
        duty_start = 0.99f;
    }
    duty_exit = duty_start - 0.05f;
    if (duty_exit < 0.40f) {
        duty_exit = 0.40f;
    }

    if (mod >= duty_start) {
        target = -current_max;
    } else if (mod <= duty_exit) {
        target = 0.0f;
    } else {
        target = fw->id;
    }

    id = fw->id;
    if (ramp_a_s <= 0.0f || dt <= 0.0f) {
        id = target;
    } else {
        step = ramp_a_s * dt;
        if (id < target) {
            id += step;
            if (id > target) {
                id = target;
            }
        } else if (id > target) {
            id -= step;
            if (id < target) {
                id = target;
            }
        }
    }
    if (id > 0.0f) {
        id = 0.0f;
    }
    if (id < -current_max) {
        id = -current_max;
    }
    fw->id = id;
    return id;
}

#ifdef __cplusplus
}
#endif
