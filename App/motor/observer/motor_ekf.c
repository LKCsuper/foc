/*
 * @Description: 扩展卡尔曼滤波无感观测器（αβ，四状态）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "motor_ekf.h"
#include "common_assert.h"
#include "motor_foc.h"

#include <math.h>
#include <stddef.h>
#include <string.h>
/* Private define ------------------------------------------------------------*/
#define MOTOR_EKF_PI (3.14159265358979323846f)
#define MOTOR_EKF_TWO_PI (2.0f * MOTOR_EKF_PI)
#define MOTOR_EKF_S_DET_MIN (1.0e-12f)
/* 状态下标：与 motor_ekf.h 中 x[] 约定一致 */
#define MOTOR_EKF_IA (0)
#define MOTOR_EKF_IB (1)
#define MOTOR_EKF_W (2)
#define MOTOR_EKF_TH (3)
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static float Motor_Ekf_WrapPi(float angle);
static float Motor_Ekf_Clamp(float value, float minimum, float maximum);
static void Motor_Ekf_ClearOutput(motor_ekf_output_t *output);
static void Motor_Ekf_Mul4(float c[MOTOR_EKF_N][MOTOR_EKF_N], const float a[MOTOR_EKF_N][MOTOR_EKF_N],
                           const float b[MOTOR_EKF_N][MOTOR_EKF_N]);
static void Motor_Ekf_Mul4Bt(float c[MOTOR_EKF_N][MOTOR_EKF_N], const float a[MOTOR_EKF_N][MOTOR_EKF_N],
                             const float b[MOTOR_EKF_N][MOTOR_EKF_N]);
static void Motor_Ekf_Symmetrize(float p[MOTOR_EKF_N][MOTOR_EKF_N]);
static void Motor_Ekf_InitP(float p[MOTOR_EKF_N][MOTOR_EKF_N]);
static void Motor_Ekf_Publish(motor_ekf_t *obs, const motor_ekf_config_t *config, motor_ekf_output_t *output);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 将角度循环归一化到 [−π, π)
 * @param {float} angle  待归一化角度 (rad)
 * @return {float} [−π, π)
 * @author: lkc
 */
static float Motor_Ekf_WrapPi(float angle)
{
    while (angle >= MOTOR_EKF_PI) {
        angle -= MOTOR_EKF_TWO_PI;
    }
    while (angle < -MOTOR_EKF_PI) {
        angle += MOTOR_EKF_TWO_PI;
    }
    return angle;
}

/**
 * @description: 将 value 限制到 [minimum, maximum]
 * @param {float} value    输入
 * @param {float} minimum  下限
 * @param {float} maximum  上限
 * @return {float} 钳位后的值
 * @author: lkc
 */
static float Motor_Ekf_Clamp(float value, float minimum, float maximum)
{
    if (value > maximum) return maximum;
    if (value < minimum) return minimum;
    return value;
}

/**
 * @description: 清空本拍输出
 * @param {motor_ekf_output_t} *output  输出；可为 NULL
 * @return {*}
 * @author: lkc
 */
static void Motor_Ekf_ClearOutput(motor_ekf_output_t *output)
{
    if (output != NULL) {
        memset(output, 0, sizeof(*output));
    }
}

/**
 * @description: 4×4 矩阵乘 C = A × B
 * @param {float} c[][]  输出
 * @param {const float} a[][]  左矩阵
 * @param {const float} b[][]  右矩阵
 * @return {*}
 * @author: lkc
 */
static void Motor_Ekf_Mul4(float c[MOTOR_EKF_N][MOTOR_EKF_N], const float a[MOTOR_EKF_N][MOTOR_EKF_N],
                           const float b[MOTOR_EKF_N][MOTOR_EKF_N])
{
    int i;
    int j;
    int k;
    float sum;

    for (i = 0; i < MOTOR_EKF_N; ++i) {
        for (j = 0; j < MOTOR_EKF_N; ++j) {
            sum = 0.0f;
            for (k = 0; k < MOTOR_EKF_N; ++k) {
                sum += a[i][k] * b[k][j];
            }
            c[i][j] = sum;
        }
    }
}

/**
 * @description: 4×4 矩阵乘 C = A × Bᵀ
 * @param {float} c[][]  输出
 * @param {const float} a[][]  左矩阵
 * @param {const float} b[][]  右矩阵（按转置参与乘法）
 * @return {*}
 * @author: lkc
 */
static void Motor_Ekf_Mul4Bt(float c[MOTOR_EKF_N][MOTOR_EKF_N], const float a[MOTOR_EKF_N][MOTOR_EKF_N],
                             const float b[MOTOR_EKF_N][MOTOR_EKF_N])
{
    int i;
    int j;
    int k;
    float sum;

    for (i = 0; i < MOTOR_EKF_N; ++i) {
        for (j = 0; j < MOTOR_EKF_N; ++j) {
            sum = 0.0f;
            for (k = 0; k < MOTOR_EKF_N; ++k) {
                sum += a[i][k] * b[j][k];
            }
            c[i][j] = sum;
        }
    }
}

/**
 * @description: 把协方差对称化，抑制欧拉积分带来的数值偏斜
 * @param {float} p[][]  协方差，就地修改
 * @return {*}
 * @author: lkc
 */
static void Motor_Ekf_Symmetrize(float p[MOTOR_EKF_N][MOTOR_EKF_N])
{
    int i;
    int j;
    float mid;

    for (i = 0; i < MOTOR_EKF_N; ++i) {
        for (j = i + 1; j < MOTOR_EKF_N; ++j) {
            mid = 0.5f * (p[i][j] + p[j][i]);
            p[i][j] = mid;
            p[j][i] = mid;
        }
    }
}

/**
 * @description: 写入 Seed 用的对角协方差
 * @param {float} p[][]  协方差
 * @return {*}
 * @author: lkc
 */
static void Motor_Ekf_InitP(float p[MOTOR_EKF_N][MOTOR_EKF_N])
{
    memset(p, 0, sizeof(float) * (size_t)MOTOR_EKF_N * (size_t)MOTOR_EKF_N);
    p[MOTOR_EKF_IA][MOTOR_EKF_IA] = 1.0e-4f;
    p[MOTOR_EKF_IB][MOTOR_EKF_IB] = 1.0e-4f;
    p[MOTOR_EKF_W][MOTOR_EKF_W] = 25.0f;
    p[MOTOR_EKF_TH][MOTOR_EKF_TH] = 0.02f;
}

/**
 * @description: 把状态写到对外角/速度和本拍输出
 * @param {motor_ekf_t} *obs                  实例
 * @param {const motor_ekf_config_t} *config  配置
 * @param {motor_ekf_output_t} *output        本拍输出
 * @return {*}
 * @author: lkc
 */
static void Motor_Ekf_Publish(motor_ekf_t *obs, const motor_ekf_config_t *config, motor_ekf_output_t *output)
{
    obs->x[MOTOR_EKF_TH] = Motor_Ekf_WrapPi(obs->x[MOTOR_EKF_TH]);
    obs->x[MOTOR_EKF_W] = Motor_Ekf_Clamp(obs->x[MOTOR_EKF_W], -MOTOR_EKF_MAX_SPEED, MOTOR_EKF_MAX_SPEED);
    obs->speed_e = obs->x[MOTOR_EKF_W];
    obs->angle = Motor_Ekf_WrapPi(obs->x[MOTOR_EKF_TH] + config->phase_offset);

    output->valid = true;
    output->speed_ok = fabsf(obs->speed_e) > MOTOR_EKF_MIN_SPEED;
    output->i_hat_alpha = obs->x[MOTOR_EKF_IA];
    output->i_hat_beta = obs->x[MOTOR_EKF_IB];
    output->angle = obs->angle;
    output->speed_e = obs->speed_e;
}

/**
 * @description: 复位 EKF 状态
 * @param {motor_ekf_t} *obs  实例，可为 NULL
 * @return {*}
 * @author: lkc
 */
void Motor_Ekf_Reset(motor_ekf_t *obs)
{
    if (obs != NULL) {
        memset(obs, 0, sizeof(*obs));
    }
}

/**
 * @description: 规整 EKF 配置
 * @param {motor_ekf_config_t} *config  配置，被原地修改；可为 NULL
 * @return {*}
 * @author: lkc
 */
void Motor_Ekf_NormalizeConfig(motor_ekf_config_t *config)
{
    COMMON_ASSERT(config != NULL);
    if (!(config->resistance >= 0.0f) || !isfinite(config->resistance)) {
        config->resistance = 0.0f;
    }
    if (!isfinite(config->inductance) || config->inductance < MOTOR_EKF_MIN_INDUCTANCE) {
        config->inductance = MOTOR_EKF_MIN_INDUCTANCE;
    }
    if (!isfinite(config->flux_linkage) || config->flux_linkage < MOTOR_EKF_MIN_LINKAGE) {
        config->flux_linkage = MOTOR_EKF_MIN_LINKAGE;
    }
    if (!isfinite(config->q_current) || config->q_current < 0.0f) {
        config->q_current = 0.0f;
    }
    if (!isfinite(config->q_speed) || config->q_speed < 0.0f) {
        config->q_speed = 0.0f;
    }
    if (!isfinite(config->q_angle) || config->q_angle < 0.0f) {
        config->q_angle = 0.0f;
    }
    if (!isfinite(config->r_current) || config->r_current < 0.0f) {
        config->r_current = 0.0f;
    }
    if (!isfinite(config->phase_offset)) {
        config->phase_offset = 0.0f;
    }
}

/**
 * @description: 用当前电流、假设电角和电角速度初始化状态与协方差
 * @param {motor_ekf_t} *obs                  实例
 * @param {const motor_ekf_config_t} *config  配置
 * @param {float} angle                       假设电角 (rad)
 * @param {float} omega_e                     假设电角速度 (rad/s)
 * @param {float} i_alpha                     α 轴电流 (A)
 * @param {float} i_beta                      β 轴电流 (A)
 * @return {*}
 * @author: lkc
 */
void Motor_Ekf_Seed(motor_ekf_t *obs, const motor_ekf_config_t *config, float angle, float omega_e, float i_alpha,
                    float i_beta)
{
    float wrapped;

    COMMON_ASSERT(obs != NULL && config != NULL);
    if (!isfinite(angle) || !isfinite(omega_e) || !isfinite(i_alpha) || !isfinite(i_beta)) return;

    wrapped = Motor_Ekf_WrapPi(angle);
    Motor_Ekf_Reset(obs);
    obs->x[MOTOR_EKF_IA] = i_alpha;
    obs->x[MOTOR_EKF_IB] = i_beta;
    obs->x[MOTOR_EKF_W] = Motor_Ekf_Clamp(omega_e, -MOTOR_EKF_MAX_SPEED, MOTOR_EKF_MAX_SPEED);
    obs->x[MOTOR_EKF_TH] = wrapped;
    Motor_Ekf_InitP(obs->p);
    obs->speed_e = obs->x[MOTOR_EKF_W];
    obs->angle = Motor_Ekf_WrapPi(wrapped + config->phase_offset);
    obs->initialized = true;
}

/**
 * @description: 推进一拍扩展卡尔曼滤波
 * @param {motor_ekf_t} *obs                  实例
 * @param {const motor_ekf_config_t} *config  配置
 * @param {float} v_alpha                     α 轴电压 (V)
 * @param {float} v_beta                      β 轴电压 (V)
 * @param {float} i_alpha                     α 轴电流 (A)
 * @param {float} i_beta                      β 轴电流 (A)
 * @param {float} dt                          控制周期 (s)
 * @param {motor_ekf_output_t} *output        本拍输出
 * @return {bool} true=本拍观测有效
 * @author: lkc
 */
bool Motor_Ekf_Step(motor_ekf_t *obs, const motor_ekf_config_t *config, float v_alpha, float v_beta, float i_alpha,
                    float i_beta, float dt, motor_ekf_output_t *output)
{
    float resistance;
    float inductance;
    float flux;
    float inv_l;
    float q_i;
    float q_w;
    float q_th;
    float r_i;
    float omega;
    float theta;
    float s_th;
    float c_th;
    float f[MOTOR_EKF_N][MOTOR_EKF_N];
    float tmp[MOTOR_EKF_N][MOTOR_EKF_N];
    float p_pred[MOTOR_EKF_N][MOTOR_EKF_N];
    float k[MOTOR_EKF_N][2];
    float a_ikh[MOTOR_EKF_N][MOTOR_EKF_N];
    float s00;
    float s01;
    float s11;
    float det;
    float inv_det;
    float inn_a;
    float inn_b;
    int i;
    int j;

    Motor_Ekf_ClearOutput(output);
    if (obs == NULL || config == NULL || output == NULL) return false;
    if (!config->enabled || !(dt > 0.0f) || !isfinite(dt) || !isfinite(v_alpha) || !isfinite(v_beta) ||
        !isfinite(i_alpha) || !isfinite(i_beta) || !(config->inductance >= MOTOR_EKF_MIN_INDUCTANCE)) {
        Motor_Ekf_Reset(obs);
        return false;
    }

    resistance = config->resistance >= 0.0f ? config->resistance : 0.0f;
    inductance = config->inductance;
    flux = config->flux_linkage >= MOTOR_EKF_MIN_LINKAGE ? config->flux_linkage : MOTOR_EKF_MIN_LINKAGE;
    inv_l = 1.0f / inductance;
    q_i = config->q_current > 0.0f ? config->q_current : MOTOR_EKF_DEFAULT_Q_CURRENT;
    q_w = config->q_speed > 0.0f ? config->q_speed : MOTOR_EKF_DEFAULT_Q_SPEED;
    q_th = config->q_angle > 0.0f ? config->q_angle : MOTOR_EKF_DEFAULT_Q_ANGLE;
    r_i = config->r_current > 0.0f ? config->r_current : MOTOR_EKF_DEFAULT_R_CURRENT;

    if (!obs->initialized) {
        Motor_Ekf_Seed(obs, config, 0.0f, 0.0f, i_alpha, i_beta);
    }

    omega = obs->x[MOTOR_EKF_W];
    theta = obs->x[MOTOR_EKF_TH];
    sincos_f32(theta, &s_th, &c_th);

    /*
     * 连续模型：
     *   L diα/dt = vα − R iα + ω ψf sinθ
     *   L diβ/dt = vβ − R iβ − ω ψf cosθ
     *   dω/dt = 0
     *   dθ/dt = ω
     * 离散 F ≈ I + dt·∂f/∂x，在预测前的 x 处线性化。
     */
    memset(f, 0, sizeof(f));
    f[MOTOR_EKF_IA][MOTOR_EKF_IA] = 1.0f - dt * resistance * inv_l;
    f[MOTOR_EKF_IA][MOTOR_EKF_W] = dt * flux * inv_l * s_th;
    f[MOTOR_EKF_IA][MOTOR_EKF_TH] = dt * flux * inv_l * omega * c_th;
    f[MOTOR_EKF_IB][MOTOR_EKF_IB] = 1.0f - dt * resistance * inv_l;
    f[MOTOR_EKF_IB][MOTOR_EKF_W] = -dt * flux * inv_l * c_th;
    f[MOTOR_EKF_IB][MOTOR_EKF_TH] = dt * flux * inv_l * omega * s_th;
    f[MOTOR_EKF_W][MOTOR_EKF_W] = 1.0f;
    f[MOTOR_EKF_TH][MOTOR_EKF_W] = dt;
    f[MOTOR_EKF_TH][MOTOR_EKF_TH] = 1.0f;

    obs->x[MOTOR_EKF_IA] += dt * inv_l * (v_alpha - resistance * obs->x[MOTOR_EKF_IA] + omega * flux * s_th);
    obs->x[MOTOR_EKF_IB] += dt * inv_l * (v_beta - resistance * obs->x[MOTOR_EKF_IB] - omega * flux * c_th);
    obs->x[MOTOR_EKF_TH] = Motor_Ekf_WrapPi(obs->x[MOTOR_EKF_TH] + omega * dt);

    Motor_Ekf_Mul4(tmp, f, obs->p);
    Motor_Ekf_Mul4Bt(p_pred, tmp, f);
    p_pred[MOTOR_EKF_IA][MOTOR_EKF_IA] += q_i;
    p_pred[MOTOR_EKF_IB][MOTOR_EKF_IB] += q_i;
    p_pred[MOTOR_EKF_W][MOTOR_EKF_W] += q_w;
    p_pred[MOTOR_EKF_TH][MOTOR_EKF_TH] += q_th;

    /* 量测 h(x)=[iα,iβ]，H P Hᵀ 就是 P 的左上 2×2。 */
    s00 = p_pred[MOTOR_EKF_IA][MOTOR_EKF_IA] + r_i;
    s01 = p_pred[MOTOR_EKF_IA][MOTOR_EKF_IB];
    s11 = p_pred[MOTOR_EKF_IB][MOTOR_EKF_IB] + r_i;
    det = s00 * s11 - s01 * s01;
    if (!(det > MOTOR_EKF_S_DET_MIN) || !isfinite(det)) {
        Motor_Ekf_Reset(obs);
        return false;
    }
    inv_det = 1.0f / det;

    /* K = P Hᵀ S⁻¹，P Hᵀ 为 P 的前两列。 */
    for (i = 0; i < MOTOR_EKF_N; ++i) {
        float p_i0 = p_pred[i][MOTOR_EKF_IA];
        float p_i1 = p_pred[i][MOTOR_EKF_IB];
        k[i][0] = (p_i0 * s11 - p_i1 * s01) * inv_det;
        k[i][1] = (-p_i0 * s01 + p_i1 * s00) * inv_det;
    }

    inn_a = i_alpha - obs->x[MOTOR_EKF_IA];
    inn_b = i_beta - obs->x[MOTOR_EKF_IB];
    for (i = 0; i < MOTOR_EKF_N; ++i) {
        obs->x[i] += k[i][0] * inn_a + k[i][1] * inn_b;
    }

    /* Joseph：P = (I−KH)P(I−KH)ᵀ + K R Kᵀ，H 只挑前两个状态。 */
    memset(a_ikh, 0, sizeof(a_ikh));
    for (i = 0; i < MOTOR_EKF_N; ++i) {
        for (j = 0; j < MOTOR_EKF_N; ++j) {
            a_ikh[i][j] = (i == j) ? 1.0f : 0.0f;
        }
        a_ikh[i][MOTOR_EKF_IA] -= k[i][0];
        a_ikh[i][MOTOR_EKF_IB] -= k[i][1];
    }
    Motor_Ekf_Mul4(tmp, a_ikh, p_pred);
    Motor_Ekf_Mul4Bt(obs->p, tmp, a_ikh);
    for (i = 0; i < MOTOR_EKF_N; ++i) {
        for (j = 0; j < MOTOR_EKF_N; ++j) {
            obs->p[i][j] += r_i * (k[i][0] * k[j][0] + k[i][1] * k[j][1]);
        }
    }
    Motor_Ekf_Symmetrize(obs->p);

    for (i = 0; i < MOTOR_EKF_N; ++i) {
        if (!isfinite(obs->x[i]) || !isfinite(obs->p[i][i])) {
            Motor_Ekf_Reset(obs);
            return false;
        }
    }

    Motor_Ekf_Publish(obs, config, output);
    if (!isfinite(obs->angle) || !isfinite(obs->speed_e)) {
        Motor_Ekf_Reset(obs);
        Motor_Ekf_ClearOutput(output);
        return false;
    }
    return true;
}

/**
 * @description: 读取观测电角
 * @param {const motor_ekf_t} *obs  实例；NULL 返回 0
 * @return {float} 电角 (rad)
 * @author: lkc
 */
float Motor_Ekf_GetAngle(const motor_ekf_t *obs)
{
    return obs == NULL ? 0.0f : obs->angle;
}

/**
 * @description: 读取估算电角速度
 * @param {const motor_ekf_t} *obs  实例；NULL 返回 0
 * @return {float} 电角速度 (rad/s)
 * @author: lkc
 */
float Motor_Ekf_GetSpeedE(const motor_ekf_t *obs)
{
    return obs == NULL ? 0.0f : obs->speed_e;
}

#ifdef __cplusplus
}
#endif
