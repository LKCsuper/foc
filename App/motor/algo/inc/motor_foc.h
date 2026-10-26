/*
 * @Description: FOC 坐标变换、SVPWM 与通信辅助函数声明
 * @Author: lkc
 */
#ifndef _MOTOR_FOC_H
#define _MOTOR_FOC_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* 本板为三电阻采样；换两电阻硬件时在编译选项中设为 0，或在板级头里 #undef 后重定义 */
#ifndef MOTOR_FOC_THREE_SHUNTS
#define MOTOR_FOC_THREE_SHUNTS  1
#endif
/* 对齐 VESC 默认 l_max_duty，满调制时仍留约 2.5% 零矢量。 */
#ifndef MOTOR_FOC_SVM_MAX_DUTY
#define MOTOR_FOC_SVM_MAX_DUTY 0.95f
#endif
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * A simple low pass filter.
 *
 * @param value
 * The filtered value.
 *
 * @param sample
 * Next sample.
 *
 * @param filter_constant
 * Filter constant. Range 0.0 to 1.0, where 1.0 gives the unfiltered value.
 */
#define UTILS_LP_FAST(value, sample, filter_constant)    (value -= (filter_constant) * ((value) - (sample)))

/**
 * A fast approximation of a moving average filter with N samples. See
 * https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average
 * https://en.wikipedia.org/wiki/Exponential_smoothing
 *
 * It is not entirely the same as it behaves like an IIR filter rather than a FIR filter, but takes
 * much less memory and is much faster to run.
 */
//#define UTILS_LP_MOVING_AVG_APPROX(value, sample, N)    UTILS_LP_FAST(value, sample, 2.0f / ((N) + 1.0f))

/**
 * @description: Clarke 变换，将三相电流变换到 αβ 静止坐标系。
 * @param {float} Ia  A 相电流 (A)
 * @param {float} Ib  B 相电流 (A)
 * @param {float} Ic  C 相电流 (A)
 * @param {float} *Ialpha  输出 α 轴电流 (A)
 * @param {float} *Ibeta   输出 β 轴电流 (A)
 * @return {void} 无
 * @author: lkc
 */
void clarke_transform(float Ia, float Ib, float Ic, float *Ialpha, float *Ibeta);

/**
 * @description: Park 变换，将 αβ 电流变换到 dq 旋转坐标系。
 * @param {float} Ialpha  α 轴电流 (A)
 * @param {float} Ibeta   β 轴电流 (A)
 * @param {float} Theta   电角度 (rad)
 * @param {float} *Id     输出 d 轴电流 (A)
 * @param {float} *Iq     输出 q 轴电流 (A)
 * @return {void} 无
 * @author: lkc
 */
void park_transform(float Ialpha, float Ibeta, float Theta, float *Id, float *Iq);

/**
 * @description: Park 变换，使用调用者已算好的同角度正弦和余弦。
 * @detail: 纯计算，无内部状态。sine/cosine 必须来自同一电角度。
 * @param {float} Ialpha  α 轴电流 (A)
 * @param {float} Ibeta   β 轴电流 (A)
 * @param {float} sine    电角度正弦
 * @param {float} cosine  电角度余弦
 * @param {float} *Id     输出 d 轴电流 (A)
 * @param {float} *Iq     输出 q 轴电流 (A)
 * @return {void} 无
 * @author: lkc
 */
void park_transform_sincos(float Ialpha, float Ibeta, float sine, float cosine, float *Id, float *Iq);

/**
 * @description: 逆 Park 变换，将 dq 电压/调制量变换到 αβ 坐标系。
 * @param {float} mod_d      d 轴分量 (V 或占空比调制量)
 * @param {float} mod_q      q 轴分量 (V 或占空比调制量)
 * @param {float} Theta      电角度 (rad)
 * @param {float} *mod_alpha 输出 α 轴分量
 * @param {float} *mod_beta  输出 β 轴分量
 * @return {void} 无
 * @author: lkc
 */
void inverse_park(float mod_d, float mod_q, float Theta, float *mod_alpha, float *mod_beta);

/**
 * @description: 逆 Park 变换，使用调用者已算好的同角度正弦和余弦。
 * @detail: 纯计算，无内部状态。极性与 inverse_park 相同。sine/cosine 必须来自同一电角度。
 * @param {float} mod_d      d 轴电压 (V)
 * @param {float} mod_q      q 轴电压 (V)
 * @param {float} sine       电角度正弦
 * @param {float} cosine     电角度余弦
 * @param {float} *mod_alpha 输出 α 轴电压 (V)
 * @param {float} *mod_beta  输出 β 轴电压 (V)
 * @return {void} 无
 * @author: lkc
 */
void inverse_park_sincos(float mod_d, float mod_q, float sine, float cosine, float *mod_alpha, float *mod_beta);

/**
 * @description: Clarke 逆变换，将 αβ 电压还原为三相相电压。
 * @param {float} Ialpha  α 轴电压 (V)
 * @param {float} Ibeta   β 轴电压 (V)
 * @param {float} *Ia     输出 A 相电压 (V)
 * @param {float} *Ib     输出 B 相电压 (V)
 * @param {float} *Ic     输出 C 相电压 (V)
 * @return {void} 无
 * @author: lkc
 */
void inverse_clarke(float Ialpha, float Ibeta, float *Ia, float *Ib, float *Ic);

/**
 * @description: 空间矢量 PWM，输出归一化三相占空比。
 * @param {float} alpha  α 轴电压分量
 * @param {float} beta   β 轴电压分量
 * @param {float} *tA    输出 A 相占空比 [0, 1]
 * @param {float} *tB    输出 B 相占空比 [0, 1]
 * @param {float} *tC    输出 C 相占空比 [0, 1]
 * @return {int} 0=成功，-1=过调制或结果非法
 * @author: lkc
 */
int svm(float alpha, float beta, float* tA, float* tB, float* tC);

/**
 * @description: 空间矢量 PWM，直接输出定时器比较值。
 * @param {float} alpha             α 轴电压分量
 * @param {float} beta              β 轴电压分量
 * @param {uint32_t} PWMFullDutyCycle  PWM 计数满量程
 * @param {uint32_t} *tAout           输出 A 相比较值
 * @param {uint32_t} *tBout           输出 B 相比较值
 * @param {uint32_t} *tCout           输出 C 相比较值
 * @param {uint32_t} *svm_sector      输出 SVPWM 扇区号
 * @return {void} 无
 * @author: lkc
 */
void foc_svm(float alpha, float beta, uint32_t PWMFullDutyCycle,
				uint32_t* tAout, uint32_t* tBout, uint32_t* tCout, uint32_t *svm_sector);

/**
 * @description: 按 VESC mcpwm_foc 把 αβ 电压变成中点 ARR/2 的三路 CCR。
 * @detail: 调制比 = 1.5 * v / v_bus（即 1/(2/3*Vbus)），再限幅到 (√3/2)*max_duty 后调用 foc_svm。
 *          零电压 CCR 为 ARR/2；max_duty 对齐 VESC l_max_duty，默认 0.95。
 * @param {float} v_alpha   α 轴电压 (V)
 * @param {float} v_beta    β 轴电压 (V)
 * @param {float} v_bus     母线电压 (V)
 * @param {float} max_duty  最大调制占空比 (0~1，无量纲)
 * @param {uint32_t} period PWM 计数满量程 (ARR)
 * @param {uint32_t} *tAout A 相 CCR
 * @param {uint32_t} *tBout B 相 CCR
 * @param {uint32_t} *tCout C 相 CCR
 * @param {uint32_t} *svm_sector SVPWM 扇区号 (1~6)
 * @return {void} 无
 * @author: lkc
 */
void foc_svm_from_voltage(float v_alpha, float v_beta, float v_bus, float max_duty, uint32_t period,
                          uint32_t *tAout, uint32_t *tBout, uint32_t *tCout, uint32_t *svm_sector);

/**
 * @description: 慧驱动 SVPWM，含母线电压限幅与最小/最大共模注入。
 * @param {float} Valpha   α 轴电压 (V)
 * @param {float} Vbeta    β 轴电压 (V)
 * @param {float} Udc      母线电压 (V)
 * @param {uint32_t} Tpwm  PWM 周期计数值
 * @param {uint32_t} *pTcmp1  输出 U 相比较值
 * @param {uint32_t} *pTcmp2  输出 V 相比较值
 * @param {uint32_t} *pTcmp3  输出 W 相比较值
 * @param {uint32_t} *pSector 输出 SVPWM 扇区号
 * @return {void} 无
 * @author: lkc
 */
void svm_hqd(float Valpha, float Vbeta, float Udc, uint32_t Tpwm, uint32_t *pTcmp1, uint32_t *pTcmp2, uint32_t *pTcmp3, uint32_t *pSector);

/**
 * @description: 查表加线性插值计算 sin(x)。
 * @param {float} x  输入角度 (rad)
 * @return {float} 正弦值
 * @author: lkc
 */
float sin_f32(float x);

/**
 * @description: 查表加线性插值计算 cos(x)。
 * @param {float} x  输入角度 (rad)
 * @return {float} 余弦值
 * @author: lkc
 */
float cos_f32(float x);
/**
 * @description: 同一角度共用正弦表索引与插值比例，计算 sin 与 cos。
 * @detail: ±64 rad 内查表插值；超范围及非有限输入回退标准库，保留异常传播。
 *          不缓存跨拍状态，不访问硬件。输出指针不可为空。
 * @param {float} x        输入角度 (rad)
 * @param {float *} sine   输出正弦值，不可为空
 * @param {float *} cosine 输出余弦值，不可为空
 * @return {void} 无
 * @author: lkc
 */
void sincos_f32(float x, float *sine, float *cosine);

/**
 * @description: 以象限缩减和泰勒多项式计算同角度正余弦，供后端对照。
 * @detail: ±64 rad 内缩减到约 ±π/4，以 Horner 形式计算 sin 七阶、cos 八阶；
 *          其他输入回退标准库，保留正负零。输出指针不可为空，不访问硬件。
 * @param {float} x 输入角度 (rad)
 * @param {float *} sine 正弦输出
 * @param {float *} cosine 余弦输出
 * @return {void} 无
 */
void sincos_taylor_f32(float x, float *sine, float *cosine);

/**
 * @description: 计算数据的 CRC-32 校验值。
 * @param {const uint8_t} *data  待校验数据指针
 * @param {uint32_t} size        数据长度 (byte)
 * @return {uint32_t} CRC-32 校验值
 * @author: lkc
 */
uint32_t crc32(const uint8_t *data, uint32_t size);

#ifdef __cplusplus
}
#endif

#endif
