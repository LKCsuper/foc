/*
 * @Description: 输出仲裁及电压重建：标准C，无HAL或全局电机依赖
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stddef.h>
#include <string.h>
#include "motor_output_core.h"
#if defined(__CC_ARM)
#pragma O2
#endif
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 按三相电流符号估计死区造成的平均电压误差，并从 α/β 电压中扣除。
 * @detail: 模型前提是电流为正时该相平均输出电压略低于比较值对应的电压，故按符号扣除；
 *          符号组合取 (2sa-sb-sc)/3 作用到 α、(sb-sc)/√3 作用到 β，与 Clarke 幅值不变型
 *          (2/3) 的约定一致，补偿幅值由 v_bus 与 fraction 共同定标。电流为零时方向不可判定，
 *          按 0 处理，静止零电流不会凭空生成补偿。纯计算，不改 PWM、不做电流滤波，
 *          也不含开关管导通压降与极性检测延迟。
 * @param {float} ia       A 相电流 (A)，仅符号参与计算
 * @param {float} ib       B 相电流 (A)，仅符号参与计算
 * @param {float} ic       C 相电流 (A)，仅符号参与计算
 * @param {float} v_bus    母线电压 (V)，有限正值
 * @param {float} fraction 死区时间占 PWM 周期的比例，无量纲，范围 [0, 0.5)
 * @param {float *} alpha  待修正的 α 轴电压 (V)，原地更新，调用者保证不可为空
 * @param {float *} beta   待修正的 β 轴电压 (V)，原地更新，调用者保证不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_OutputCore_SubtractDeadtime(float ia, float ib, float ic, float v_bus,
                                         float fraction, float *alpha, float *beta)
{
    int sa = (ia > 0.0f) - (ia < 0.0f);
    int sb = (ib > 0.0f) - (ib < 0.0f);
    int sc = (ic > 0.0f) - (ic < 0.0f);
    float scale = (2.0f / 3.0f) * v_bus * fraction;
    /* 符号组合只有小整数；合并后再转换，固定除数改为倒数乘法。 */
    *alpha -= (float)(2 * sa - sb - sc) * (1.0f / 3.0f) * scale;
    *beta -= (float)(sb - sc) * 0.577350269f * scale;
}

/**
 * @description: 由三相比较值与母线电压重建平均 α/β 电压。
 * @detail: 纯计算，不访问硬件；duty1~3 为逻辑 ABC 比较值，按 period 满量程换算为相电压，
 *          α/β 取 Clarke 幅值不变型 (2/3) 组合。deadtime_fraction 非零时再按相电流符号做死区
 *          修正，此时才要求 ia/ib/ic 有限；为零时跳过修正，电流字段无需有效。
 *          重建结果是平均电压估计，不能直接当作 PWM 命令使用。
 * @param {const motor_output_sample_t *} motor 采样快照，读取 duty1~3 与 ia/ib/ic，不可为空
 * @param {uint32_t} period                  PWM 周期计数，须大于 0
 * @param {float} v_bus                      母线电压 (V)，须为有限正值
 * @param {float} deadtime_fraction          死区补偿比例，无量纲，范围 [0, 0.5)，0 表示不补偿
 * @param {float *} v_alpha                  输出 α 轴电压 (V)，不可为空
 * @param {float *} v_beta                   输出 β 轴电压 (V)，不可为空
 * @return {bool} 参数合法并完成重建返回 true；参数非法时清零非空输出并返回 false
 * @author: lkc
 */
bool Motor_OutputCore_EstimateVoltage(const motor_output_sample_t *motor, uint32_t period, float v_bus,
                                 float deadtime_fraction, float *v_alpha, float *v_beta)
{
    float scale;

    if (v_alpha != NULL) *v_alpha = 0.0f;
    if (v_beta != NULL) *v_beta = 0.0f;
    if (motor == NULL || v_alpha == NULL || v_beta == NULL || period == 0U ||
        !isfinite(v_bus) || v_bus <= 0.0f || !isfinite(deadtime_fraction) ||
        deadtime_fraction < 0.0f || deadtime_fraction >= 0.5f ||
        motor->duty1 > period || motor->duty2 > period || motor->duty3 > period) {
        return false;
    }
    if (deadtime_fraction > 0.0f &&
        (!isfinite(motor->ia) || !isfinite(motor->ib) || !isfinite(motor->ic))) {
        return false;
    }
    scale = v_bus / (float)period;
    *v_alpha = (2.0f * motor->duty1 - motor->duty2 - motor->duty3) * (1.0f / 3.0f) * scale;
    *v_beta = ((float)motor->duty2 - motor->duty3) * 0.577350269f * scale;
    if (deadtime_fraction > 0.0f) {
        /* VESC调制以1.5/Vbus归一化；补偿转回SI电压须乘2/3*Vbus。
         * 零流方向不可判定，按0处理，避免静止零电流凭空生成补偿。 */
        Motor_OutputCore_SubtractDeadtime(motor->ia, motor->ib, motor->ic, v_bus,
                                     deadtime_fraction, v_alpha, v_beta);
    }
    return true;
}

/**
 * @description: 本拍输出仲裁：已锁存故障时丢弃控制期望，统一给出 SAFE。
 * @detail: 纯逻辑，不阻塞、不访问硬件。SAFE 为三相中点占空比，只表示本拍不施加主动电压矢量，
 *          不等于关桥，也不保证无转矩；控制期望为空且无故障时返回 NONE（本拍不更新 PWM）。
 * @param {bool} fault_latched 保护锁存状态，true 时优先返回 SAFE
 * @param {const motor_control_result_t *} proposed 本拍控制期望，允许为 NULL
 * @return {motor_control_result_t} 本拍最终输出动作与 d/q 电压矢量；SAFE/NONE 时其余字段清零
 * @author: lkc
 */
motor_control_result_t Motor_Control_ArbitrateOutput(bool fault_latched, const motor_control_result_t *proposed)
{
    motor_control_result_t none;

    if (fault_latched) {
        memset(&none, 0, sizeof(none));
        none.action = MOTOR_CONTROL_ACTION_SAFE;
        return none;
    }
    if (proposed == NULL) {
        memset(&none, 0, sizeof(none));
        none.action = MOTOR_CONTROL_ACTION_NONE;
        return none;
    }
    return *proposed;
}

#ifdef __cplusplus
}
#endif
