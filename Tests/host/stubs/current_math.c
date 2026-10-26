/*
 * Clarke/Park stubs for motor_current host tests.
 */
/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include "motor_foc.h"

/* Private functions ---------------------------------------------------------*/
/**
 * @description: 主机测试替身：查表加线性插值计算 sin(x)。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} angle angle 参数
 * @return {float} 正弦值
 */
float sin_f32(float angle) { return sinf(angle); }
/**
 * @description: 主机测试替身：查表 + 线性插值计算 cos(x)
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} angle angle 参数
 * @return {float} * @author: lkc
 */
float cos_f32(float angle) { return cosf(angle); }
/**
 * @description: 主机测试替身：同一角度共用正弦表索引与插值比例，计算 sin 与 cos。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} angle angle 参数
 * @param {float *} sine sine 参数
 * @param {float *} cosine cosine 参数
 * @return {void} 无
 */
void sincos_f32(float angle, float *sine, float *cosine) { *sine = sinf(angle); *cosine = cosf(angle); }

#ifndef ONE_BY_SQRT3
#define ONE_BY_SQRT3 (0.57735026919f)
#endif

/**
 * @description: 主机测试替身：Clarke 变换，将三相电流投影到 αβ 静止坐标系。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} Ia Ia 参数
 * @param {float} Ib Ib 参数
 * @param {float} Ic Ic 参数
 * @param {float *} Ialpha Ialpha 参数
 * @param {float *} Ibeta Ibeta 参数
 * @return {void} 无
 */
void clarke_transform(float Ia, float Ib, float Ic, float *Ialpha, float *Ibeta)
{
    *Ialpha = (2.0f / 3.0f) * Ia - (1.0f / 3.0f) * Ib - (1.0f / 3.0f) * Ic;
    *Ibeta = (Ib - Ic) * ONE_BY_SQRT3;
}

/**
 * @description: 主机测试替身：Park 变换，将 αβ 电流投影到与转子同步的 dq 坐标系。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} Ialpha Ialpha 参数
 * @param {float} Ibeta Ibeta 参数
 * @param {float} Theta Theta 参数
 * @param {float *} Id Id 参数
 * @param {float *} Iq Iq 参数
 * @return {void} 无
 */
void park_transform(float Ialpha, float Ibeta, float Theta, float *Id, float *Iq)
{
    float s = sinf(Theta);
    float c = cosf(Theta);

    *Id = Ialpha * c + Ibeta * s;
    *Iq = -Ialpha * s + Ibeta * c;
}

/**
 * @description: 主机测试替身：Park 变换，使用调用者已算好的同角度正弦和余弦。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} Ialpha Ialpha 参数
 * @param {float} Ibeta Ibeta 参数
 * @param {float} sine sine 参数
 * @param {float} cosine cosine 参数
 * @param {float *} Id Id 参数
 * @param {float *} Iq Iq 参数
 * @return {void} 无
 */
void park_transform_sincos(float Ialpha, float Ibeta, float sine, float cosine, float *Id, float *Iq)
{
    *Id = Ialpha * cosine + Ibeta * sine;
    *Iq = -Ialpha * sine + Ibeta * cosine;
}
#include "motor_timing.h"
/**
 * @description: 主机测试替身：返回本拍 Begin 写下的采样标志。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 本实现固定返回 false
 */
bool Motor_Timing_ProfileIsActive(void) { return false; }
/**
 * @description: 主机测试替身：判断本电流控制拍是否应记录交接样本。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 本实现固定返回 false
 */
bool Motor_Timing_HandoffDue(void) { return false; }
/**
 * @description: 主机测试替身：写入交接拍限幅前的电流与积分。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {const MOTOR_PARAM_S *} motor 兼容参数，本实现不使用
 * @param {float} theta 兼容参数，本实现不使用
 * @param {float} delta 兼容参数，本实现不使用
 * @param {float} id 兼容参数，本实现不使用
 * @param {float} iq 兼容参数，本实现不使用
 * @param {float} dt 兼容参数，本实现不使用
 * @return {void} 无
 */
void Motor_Timing_HandoffBefore(const MOTOR_PARAM_S *motor, float theta, float delta, float id, float iq, float dt)
{ (void)motor; (void)theta; (void)delta; (void)id; (void)iq; (void)dt; }
/**
 * @description: 主机测试替身：写入坐标旋转后的电流积分。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {const MOTOR_PARAM_S *} motor 兼容参数，本实现不使用
 * @return {void} 无
 */
void Motor_Timing_HandoffTransport(const MOTOR_PARAM_S *motor) { (void)motor; }
/**
 * @description: 主机测试替身：写入限幅后的积分与电压，并推进环形索引。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {const MOTOR_PARAM_S *} motor 兼容参数，本实现不使用
 * @param {float} vd 兼容参数，本实现不使用
 * @param {float} vq 兼容参数，本实现不使用
 * @return {void} 无
 */
void Motor_Timing_HandoffAfter(const MOTOR_PARAM_S *motor, float vd, float vq)
{ (void)motor; (void)vd; (void)vq; }
/**
 * @description: 主机测试替身：查询本拍细分计时是否有效。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 本实现固定返回 false
 */
bool Motor_Timing_ProfileDetailedIsActive(void) { return false; }
/**
 * @description: 主机测试替身：读取分段计时用的当前时间戳。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {uint32_t} 本实现固定返回 0U
 */
uint32_t Motor_Timing_ProfileTimeUs(void) { return 0U; }
/**
 * @description: 主机测试替身：把一个阶段从 start_us 到现在的耗时累进当前窗口。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {motor_timing_stage_t} stage 兼容参数，本实现不使用
 * @param {uint32_t} start 兼容参数，本实现不使用
 * @return {void} 无
 */
void Motor_Timing_ProfileRecord(motor_timing_stage_t stage, uint32_t start) { (void)stage; (void)start; }
