/* 输出派发主机测试：三相电压估计、SVM 路径选择、观测死区补偿与 Park 复用 */
/* Includes ------------------------------------------------------------------*/
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_output.h"
#include "motor_feedback.h"
#include "motor_timing.h"

static motor_feedback_source_t s_source;
static unsigned s_vesc_calls;
static unsigned s_hqd_calls;
static unsigned s_commit_calls;
static uint32_t s_pwm_a;
static uint32_t s_pwm_b;
static uint32_t s_pwm_c;
static float s_applied_alpha;
static float s_applied_beta;
static float s_vesc_alpha;
static float s_vesc_beta;
static float s_reconstructed_alpha;
static float s_reconstructed_beta;
static unsigned s_reconstruction_calls;
static float s_command_alpha;
static float s_command_beta;
static bool s_profile_detailed;
static unsigned s_voltage_records;

/* Private functions ---------------------------------------------------------*/
/**
 * @description: 主机测试替身：查询本拍细分计时是否有效。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 测试记录值 s_profile_detailed
 */
bool Motor_Timing_ProfileDetailedIsActive(void) { return s_profile_detailed; }
/**
 * @description: 主机测试替身：读取分段计时用的当前时间戳。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {uint32_t} 本实现固定返回 0U
 */
uint32_t Motor_Timing_ProfileTimeUs(void) { return 0U; }
/**
 * @description: 主机测试替身：把一个阶段从 start_us 到现在的耗时累进当前窗口。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {motor_timing_stage_t} stage 检测阶段
 * @param {uint32_t} start_us 兼容参数，本实现不使用
 * @return {void} 无
 */
void Motor_Timing_ProfileRecord(motor_timing_stage_t stage, uint32_t start_us)
{
    (void)start_us;
    if (stage == MOTOR_TIMING_VOLTAGE_REBUILD) s_voltage_records++;
}

/**
 * @description: 主机测试替身：读取 PWM 周期计数（通常等于 ARR）
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {uint32_t} 测试记录值 1000U
 */
uint32_t Hal_Motor_PwmPeriodCounts(void) { return 1000U; }
/**
 * @description: 主机测试替身：设置三相 PWM 预装载比较值。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {uint32_t} a a 参数
 * @param {uint32_t} b b 参数
 * @param {uint32_t} c c 参数
 * @return {void} 无
 */
void Hal_Motor_PwmSetDuty(uint32_t a, uint32_t b, uint32_t c)
{
    s_pwm_a = a;
    s_pwm_b = b;
    s_pwm_c = c;
}
/**
 * @description: 主机测试替身：提交 PWM 通道控制更新。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
void Hal_Motor_PwmCommit(void) { s_commit_calls++; }
/**
 * @description: 主机测试替身：读取默认单实例请求的反馈来源
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {motor_feedback_source_t} 测试记录值 s_source
 */
motor_feedback_source_t Motor_Feedback_GetSourceDefault(void) { return s_source; }
/**
 * @description: 主机测试替身：记录默认实例本周期实际下发的 α/β 电压。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} alpha 输出静止坐标 alpha 轴电压 (V)
 * @param {float} beta 输出静止坐标 beta 轴电压 (V)
 * @return {void} 无
 */
void Motor_Sensorless_SetAppliedVoltage(float alpha, float beta)
{
    s_applied_alpha = alpha;
    s_applied_beta = beta;
}
/**
 * @description: 主机测试替身：记录最终 PWM 平均电压，仅用于和磁链积分电压比较。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} alpha 输出静止坐标 alpha 轴电压 (V)
 * @param {float} beta 输出静止坐标 beta 轴电压 (V)
 * @return {void} 无
 */
void Motor_Sensorless_SetModulatedVoltage(float alpha, float beta)
{
    s_command_alpha = s_applied_alpha;
    s_command_beta = s_applied_beta;
    s_reconstructed_alpha = alpha;
    s_reconstructed_beta = beta;
    s_reconstruction_calls++;
}
static unsigned s_inverse_park_calls;
static unsigned s_inverse_park_sincos_calls;
static float s_reuse_sine;
static float s_reuse_cosine;
/**
 * @description: 主机测试替身：逆 Park 变换，将 dq 电压/调制量变换到 αβ 坐标系。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} vd D 轴电压 (V)
 * @param {float} vq Q 轴电压 (V)
 * @param {float} theta 兼容参数，本实现不使用
 * @param {float *} alpha 输出静止坐标 alpha 轴电压 (V)
 * @param {float *} beta 输出静止坐标 beta 轴电压 (V)
 * @return {void} 无
 */
void inverse_park(float vd, float vq, float theta, float *alpha, float *beta)
{
    (void)theta;
    s_inverse_park_calls++;
    *alpha = vd;
    *beta = vq;
}
/**
 * @description: 主机测试替身：逆 Park 变换，使用调用者已算好的同角度正弦和余弦。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} vd D 轴电压 (V)
 * @param {float} vq Q 轴电压 (V)
 * @param {float} sine sine 参数
 * @param {float} cosine cosine 参数
 * @param {float *} alpha 输出静止坐标 alpha 轴电压 (V)
 * @param {float *} beta 输出静止坐标 beta 轴电压 (V)
 * @return {void} 无
 */
void inverse_park_sincos(float vd, float vq, float sine, float cosine, float *alpha, float *beta)
{
    s_inverse_park_sincos_calls++;
    s_reuse_sine = sine;
    s_reuse_cosine = cosine;
    *alpha = vd;
    *beta = vq;
}
/**
 * @description: 由测试电压及母线条件生成 SVM 比较值。
 * @param {float} alpha 输出静止坐标 alpha 轴电压 (V)
 * @param {float} beta 输出静止坐标 beta 轴电压 (V)
 * @param {float} v_bus 兼容参数，本实现不使用
 * @param {float} max_duty 兼容参数，本实现不使用
 * @param {uint32_t} period 兼容参数，本实现不使用
 * @param {uint32_t *} a a 参数
 * @param {uint32_t *} b b 参数
 * @param {uint32_t *} c c 参数
 * @param {uint32_t *} sector sector 参数
 * @return {void} 无
 */
void foc_svm_from_voltage(float alpha, float beta, float v_bus, float max_duty, uint32_t period,
                          uint32_t *a, uint32_t *b, uint32_t *c, uint32_t *sector)
{
    (void)v_bus;
    (void)max_duty;
    (void)period;
    s_vesc_alpha = alpha;
    s_vesc_beta = beta;
    s_vesc_calls++;
    *a = 11U;
    *b = 22U;
    *c = 33U;
    *sector = 1U;
}
/**
 * @description: 主机测试替身：慧驱动 SVPWM，将 αβ 电压矢量转为三路 PWM 比较值。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} alpha 兼容参数，本实现不使用
 * @param {float} beta 兼容参数，本实现不使用
 * @param {float} v_bus 兼容参数，本实现不使用
 * @param {uint32_t} period 兼容参数，本实现不使用
 * @param {uint32_t *} a a 参数
 * @param {uint32_t *} b b 参数
 * @param {uint32_t *} c c 参数
 * @param {uint32_t *} sector sector 参数
 * @return {void} 无
 */
void svm_hqd(float alpha, float beta, float v_bus, uint32_t period,
             uint32_t *a, uint32_t *b, uint32_t *c, uint32_t *sector)
{
    (void)alpha;
    (void)beta;
    (void)v_bus;
    (void)period;
    s_hqd_calls++;
    *a = 101U;
    *b = 202U;
    *c = 303U;
    *sector = 2U;
}

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    MOTOR_PARAM_S motor;
    motor_control_result_t result;

    memset(&motor, 0, sizeof(motor));
    /* 独立构造三相平均端电压：验证相序、公共模抵消及补偿方向。 */
    {
        float alpha;
        float beta;
        motor.duty1 = 600U;
        motor.duty2 = 450U;
        motor.duty3 = 300U;
        assert(Motor_OutputEstimateVoltage(&motor, 1000U, 24.0f, 0.0f, &alpha, &beta));
        assert(fabsf(alpha - 3.6f) < 1e-5f);
        assert(fabsf(beta - 2.07846097f) < 1e-5f);
        motor.duty1 += 100U;
        motor.duty2 += 100U;
        motor.duty3 += 100U;
        assert(Motor_OutputEstimateVoltage(&motor, 1000U, 24.0f, 0.0f, &alpha, &beta));
        assert(fabsf(alpha - 3.6f) < 1e-5f && fabsf(beta - 2.07846097f) < 1e-5f);
        motor.duty1 = motor.duty2 = motor.duty3 = 500U;
        motor.ia = 1.0f;
        motor.ib = motor.ic = -0.5f;
        assert(Motor_OutputEstimateVoltage(&motor, 1000U, 24.0f, 0.01f, &alpha, &beta));
        assert(fabsf(alpha + 0.21333333f) < 1e-5f && beta == 0.0f);
        motor.ia = -1.0f;
        motor.ib = motor.ic = 0.5f;
        assert(Motor_OutputEstimateVoltage(&motor, 1000U, 24.0f, 0.01f, &alpha, &beta));
        assert(fabsf(alpha - 0.21333333f) < 1e-5f && beta == 0.0f);
        motor.ia = 0.0f;
        motor.ib = 1.0f;
        motor.ic = -1.0f;
        assert(Motor_OutputEstimateVoltage(&motor, 1000U, 24.0f, 0.01f, &alpha, &beta));
        assert(alpha == 0.0f && fabsf(beta + 0.18475209f) < 1e-5f);
        motor.ib = motor.ic = 0.0f;
        assert(Motor_OutputEstimateVoltage(&motor, 1000U, 24.0f, 0.01f, &alpha, &beta));
        assert(alpha == 0.0f && beta == 0.0f);
        assert(!Motor_OutputEstimateVoltage(&motor, 0U, 24.0f, 0.0f, &alpha, &beta));
        assert(!Motor_OutputEstimateVoltage(&motor, 1000U, NAN, 0.0f, &alpha, &beta));
        assert(!Motor_OutputEstimateVoltage(&motor, 1000U, 24.0f, -0.01f, &alpha, &beta));
        motor.duty1 = 1001U;
        assert(!Motor_OutputEstimateVoltage(&motor, 1000U, 24.0f, 0.0f, &alpha, &beta));
        assert(alpha == 0.0f && beta == 0.0f);
        memset(&motor, 0, sizeof(motor));
    }
    Motor_OutputSafe(&motor);
    assert(motor.duty1 == 500U && motor.duty2 == 500U && motor.duty3 == 500U);
    assert(s_pwm_a == 500U && s_pwm_b == 500U && s_pwm_c == 500U);
    assert(s_applied_alpha == 0.0f && s_applied_beta == 0.0f && s_commit_calls == 1U);

    motor.ol_mode = MOTOR_OL_MODE_IF;
    motor.sensorless_enable = true;
    motor.sensorless_takeover = true;
    s_source = MOTOR_FEEDBACK_SRC_ENCODER;
    Motor_OutputVoltage(&motor, 2.0f, -3.0f, 1.0f, 24.0f);
    assert(s_vesc_calls == 1U && s_hqd_calls == 0U);
    assert(motor.duty1 == 11U && motor.duty2 == 22U && motor.duty3 == 33U);
    assert(s_pwm_a == 11U && s_pwm_b == 33U && s_pwm_c == 22U);
    assert(s_applied_alpha == 2.0f && s_applied_beta == -3.0f);
    assert(s_vesc_alpha == -2.0f && s_vesc_beta == 3.0f);

    s_source = MOTOR_FEEDBACK_SRC_SENSORLESS;
    motor.ol_mode = MOTOR_OL_MODE_IF;
    Motor_OutputVoltage(&motor, 2.0f, -3.0f, 1.0f, 24.0f);
    assert(s_vesc_calls == 2U && s_hqd_calls == 0U);
    assert(fabsf(s_applied_alpha + 0.264f) < 1e-5f);
    assert(fabsf(s_applied_beta + 0.15242047f) < 1e-5f);
    assert(s_vesc_alpha == -2.0f && s_vesc_beta == 3.0f);
    assert(s_reconstruction_calls == 1U);
    /* 逻辑ABC=11/22/33，保持β为负，不因物理ACB映射而反号。 */
    assert(fabsf(s_reconstructed_alpha + 0.264f) < 1e-5f);
    assert(fabsf(s_reconstructed_beta + 0.15242047f) < 1e-5f);

    s_source = MOTOR_FEEDBACK_SRC_OPENLOOP;
    Motor_OutputVoltage(&motor, 2.0f, -3.0f, 1.0f, 24.0f);
    assert(s_vesc_calls == 3U && s_hqd_calls == 0U);
    assert(fabsf(s_applied_alpha + 0.264f) < 1e-5f);
    assert(fabsf(s_applied_beta + 0.15242047f) < 1e-5f);

    motor.ol_mode = MOTOR_OL_MODE_VF;
    s_source = MOTOR_FEEDBACK_SRC_OPENLOOP;
    result.action = MOTOR_CONTROL_ACTION_OUTPUT_DQ;
    result.vd = 1.0f;
    result.vq = 4.0f;
    result.theta = 0.5f;
    Motor_OutputApplyControlResult(&motor, 24.0f, &result);
    assert(s_vesc_calls == 3U && s_hqd_calls == 1U);
    assert(s_applied_alpha == 1.0f && s_applied_beta == 4.0f);
    assert(s_pwm_a == 101U && s_pwm_b == 303U && s_pwm_c == 202U);
    assert(s_reconstruction_calls == 2U);

    result.action = MOTOR_CONTROL_ACTION_NONE;
    Motor_OutputApplyControlResult(&motor, 24.0f, &result);
    assert(s_commit_calls == 5U);
    /* 观测补偿只改变估计电压，不能改变比较值；无效设置不能覆盖已选参数。 */
    assert(Motor_OutputGetObserverDeadtime() == 0.0f);
    assert(Motor_OutputSetObserverDeadtime(0.0054f));
    assert(!Motor_OutputSetObserverDeadtime(NAN));
    assert(!Motor_OutputSetObserverDeadtime(-0.001f));
    assert(!Motor_OutputSetObserverDeadtime(0.021f));
    assert(Motor_OutputGetObserverDeadtime() == 0.0054f);
    motor.ol_mode = MOTOR_OL_MODE_IF;
    s_source = MOTOR_FEEDBACK_SRC_SENSORLESS;
    motor.ia = 1.0f;
    motor.ib = motor.ic = -0.5f;
    Motor_OutputVoltage(&motor, 2.0f, -3.0f, 1.0f, 24.0f);
    assert(s_pwm_a == 11U && s_pwm_b == 33U && s_pwm_c == 22U);
    assert(fabsf(s_applied_alpha - (s_reconstructed_alpha - 0.1152f)) < 1e-5f);
    assert(fabsf(s_applied_beta - s_reconstructed_beta) < 1e-5f);
    /* 单次电流噪声反向不应立即翻转补偿方向。 */
    motor.ia = -0.01f;
    motor.ib = motor.ic = 0.005f;
    Motor_OutputVoltage(&motor, 2.0f, -3.0f, 1.0f, 24.0f);
    assert(fabsf(s_applied_alpha - (s_reconstructed_alpha - 0.1152f)) < 1e-5f);
    Motor_OutputSafe(&motor);
    motor.ia = -1.0f;
    motor.ib = motor.ic = 0.5f;
    Motor_OutputVoltage(&motor, 2.0f, -3.0f, 1.0f, 24.0f);
    assert(fabsf(s_applied_alpha - (s_reconstructed_alpha + 0.1152f)) < 1e-5f);
    assert(Motor_OutputSetObserverDeadtime(0.0f));
    Motor_OutputVoltage(&motor, 2.0f, -3.0f, 1.0f, 24.0f);
    assert(s_applied_alpha == s_reconstructed_alpha && s_applied_beta == s_reconstructed_beta);
    /* 显式路径不受模式/反馈元数据影响；检查符号、相序、诊断顺序及 SAFE/NONE。 */
    {
        unsigned commits = s_commit_calls;
        unsigned reconstructions = s_reconstruction_calls;
        unsigned hqd = s_hqd_calls;
        unsigned vesc = s_vesc_calls;
        result.action = MOTOR_CONTROL_ACTION_OUTPUT_DQ;
        result.vd = 2.0f;
        result.vq = -3.0f;
        result.theta = 1.0f;
        motor.ol_mode = MOTOR_OL_MODE_VF;
        s_source = MOTOR_FEEDBACK_SRC_ENCODER;
        Motor_OutputApplyControlResultForPath(&motor, 24.0f, &result, MOTOR_OUTPUT_PATH_SENSORLESS_IF);
        assert(s_vesc_calls == vesc + 1U && s_hqd_calls == hqd);
        assert(s_reconstruction_calls == reconstructions + 1U);
        assert(s_command_alpha == -2.0f && s_command_beta == 3.0f);
        assert(s_applied_alpha == s_reconstructed_alpha && s_applied_beta == s_reconstructed_beta);
        assert(s_pwm_a == 11U && s_pwm_b == 33U && s_pwm_c == 22U);
        motor.ol_mode = MOTOR_OL_MODE_IF;
        s_source = MOTOR_FEEDBACK_SRC_SENSORLESS;
        Motor_OutputApplyControlResultForPath(&motor, 24.0f, &result, MOTOR_OUTPUT_PATH_VF);
        assert(s_hqd_calls == hqd + 1U && s_vesc_calls == vesc + 1U);
        assert(s_applied_alpha == 2.0f && s_applied_beta == -3.0f);
        Motor_OutputApplyControlResultForPath(&motor, 24.0f, &result, MOTOR_OUTPUT_PATH_SENSORED_IF);
        assert(s_vesc_calls == vesc + 2U && s_hqd_calls == hqd + 1U);
        assert(s_applied_alpha == 2.0f && s_applied_beta == -3.0f);
        assert(s_reconstruction_calls == reconstructions + 1U);
        assert(s_commit_calls == commits + 3U);
        result.action = MOTOR_CONTROL_ACTION_NONE;
        Motor_OutputApplyControlResultForPath(&motor, 24.0f, &result, MOTOR_OUTPUT_PATH_SENSORLESS_IF);
        assert(s_commit_calls == commits + 3U);
        result.action = MOTOR_CONTROL_ACTION_SAFE;
        Motor_OutputApplyControlResultForPath(&motor, 24.0f, &result, MOTOR_OUTPUT_PATH_SENSORLESS_IF);
        assert(s_pwm_a == 500U && s_pwm_b == 500U && s_pwm_c == 500U);
        result.action = MOTOR_CONTROL_ACTION_OUTPUT_DQ;
        Motor_OutputApplyControlResultForPath(&motor, 24.0f, &result, (motor_output_path_t)99);
        assert(s_pwm_a == 500U && s_pwm_b == 500U && s_pwm_c == 500U);
        motor.ol_mode = MOTOR_OL_MODE_VF;
        Motor_OutputVoltageLinear(&motor, 2.0f, -3.0f, 1.0f, 24.0f);
        assert(s_vesc_calls == vesc + 3U && s_hqd_calls == hqd + 1U);
        assert(s_applied_alpha == 2.0f && s_applied_beta == -3.0f);
        motor.ol_mode = MOTOR_OL_MODE_IF;
        Motor_OutputVoltageLinear(&motor, 2.0f, -3.0f, 1.0f, 24.0f);
        assert(s_reconstruction_calls == reconstructions + 2U);
    }
    /* 电压重建只计入使能的无感 I/F 细分拍，开环和停机仍重建但不进该段。 */
    s_profile_detailed = true;
    motor.ol_enable = true;
    motor.sensorless_enable = true;
    motor.ol_mode = MOTOR_OL_MODE_IF;
    s_source = MOTOR_FEEDBACK_SRC_SENSORLESS;
    s_voltage_records = 0U;
    Motor_OutputVoltage(&motor, 2.0f, -3.0f, 1.0f, 24.0f);
    assert(s_voltage_records == 1U);
    s_source = MOTOR_FEEDBACK_SRC_OPENLOOP;
    Motor_OutputVoltage(&motor, 2.0f, -3.0f, 1.0f, 24.0f);
    assert(s_voltage_records == 1U);
    s_source = MOTOR_FEEDBACK_SRC_SENSORLESS;
    motor.ol_enable = false;
    Motor_OutputVoltage(&motor, 2.0f, -3.0f, 1.0f, 24.0f);
    assert(s_voltage_records == 1U);
    motor.ol_enable = true;
    s_profile_detailed = false;
    Motor_OutputVoltage(&motor, 2.0f, -3.0f, 1.0f, 24.0f);
    assert(s_voltage_records == 1U);
    /* 仅观测时仍重建电压，比较值和 SVM 输入保持原符号。 */
    motor.sensorless_takeover = false;
    motor.ol_enable = true;
    motor.ol_mode = MOTOR_OL_MODE_IF;
    s_source = MOTOR_FEEDBACK_SRC_SENSORLESS;
    s_profile_detailed = true;
    {
        unsigned reconstructions = s_reconstruction_calls;
        unsigned records = s_voltage_records;
        Motor_OutputVoltage(&motor, 2.0f, -3.0f, 1.0f, 24.0f);
        assert(s_reconstruction_calls == reconstructions + 1U);
        assert(s_voltage_records == records + 1U);
        assert(s_pwm_a == 11U && s_pwm_b == 33U && s_pwm_c == 22U);
        assert(s_command_alpha == -2.0f && s_command_beta == 3.0f);
        assert(s_vesc_alpha == -2.0f && s_vesc_beta == 3.0f);
        motor.sensorless_enable = false;
        Motor_OutputVoltage(&motor, 2.0f, -3.0f, 1.0f, 24.0f);
        assert(s_reconstruction_calls == reconstructions + 1U);
        assert(s_voltage_records == records + 1U);
        assert(s_applied_alpha == -2.0f && s_applied_beta == 3.0f);
        motor.sensorless_enable = true;
    }
    /* 同实例 Park 角度原码相同才复用正余弦；角度不同、正负零不同或未做过 Park 都按 theta 重算。 */
    {
        unsigned direct = s_inverse_park_calls;
        unsigned reused = s_inverse_park_sincos_calls;
        motor.ol_mode = MOTOR_OL_MODE_VF;
        motor.park_theta = 1.0f;
        motor.park_sin = 0.25f;
        motor.park_cos = 0.75f;
        motor.park_sincos_valid = true;
        Motor_OutputVoltage(&motor, 2.0f, -3.0f, 1.0f, 24.0f);
        assert(s_inverse_park_sincos_calls == reused + 1U && s_inverse_park_calls == direct);
        assert(s_reuse_sine == 0.25f && s_reuse_cosine == 0.75f);
        Motor_OutputVoltage(&motor, 2.0f, -3.0f, 1.5f, 24.0f);
        assert(s_inverse_park_calls == direct + 1U);
        motor.park_theta = 0.0f;
        Motor_OutputVoltage(&motor, 2.0f, -3.0f, -0.0f, 24.0f);
        assert(s_inverse_park_calls == direct + 2U);
        motor.park_theta = 1.0f;
        motor.park_sincos_valid = false;
        Motor_OutputVoltage(&motor, 2.0f, -3.0f, 1.0f, 24.0f);
        assert(s_inverse_park_calls == direct + 3U && s_inverse_park_sincos_calls == reused + 1U);
    }
    puts("test_motor_output: pass");
    return 0;
}
