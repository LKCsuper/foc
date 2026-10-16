/*
 * @Description: 编码器角度、速度与零点（MT6701 绝对计数 + ODrive 临界阻尼 PLL）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include "motor_encoder.h"
#include "motor_param.h"
#include "motor_math.h"
#include "motor.h"
#include "hal_sensor.h"
#include "motor_mt6701.h"
/* Private define ------------------------------------------------------------*/
#ifndef MOTOR_ENCODER_INVERTED
#define MOTOR_ENCODER_INVERTED (false)
#endif
#define MOTOR_ENCODER_DT_MIN_S (1.0e-6f)
#define MOTOR_ENCODER_DT_MAX_S (0.1f)
/*
 * MT6701 14 bit / 10 kHz 在低速时每帧只有 0/1 count 变化。
 * 带宽由电机头提供：小电机低速齿槽明显时使用较低带宽，大电机保留较快响应。
 * PLL 只估算速度；FOC 电角度仍直接取最新 MT6701 绝对计数，不受该带宽影响。
 */
#define MOTOR_ENCODER_MAX_RPM (55000.0f)
#define MOTOR_ENCODER_JUMP_MARGIN_COUNTS (2.0f)
#define MOTOR_ENCODER_SAMPLE_TIMEOUT_S (0.002f)
#define MOTOR_ENCODER_CORRECTION_DT_MAX_S (0.0004f)
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static motor_encoder_runtime_t s_encoder;                      /* 默认实例：零点、方向、PLL 与速度历史 */
static volatile bool s_encoder_compare_enabled;                /* 编码器对照采集开关（诊断用，不影响控制反馈） */
static motor_encoder_sampling_diagnostics_t s_sampling_diagnostics; /* 累计采样诊断：帧数、最大间隔、过期重置次数 */
static uint32_t s_encoder_sample_sequence;                     /* 最近一次已消费帧的序号，用于判断是否为新帧 */
static uint32_t s_encoder_sample_time_us;                      /* 最近一次已消费帧的时间戳 (us) */
static bool s_encoder_have_sequence;                           /* 是否已消费过至少一帧；false 时按首次采样处理 */
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 查询编码器对照采集是否打开。
 * @return {bool} true=对照采集已打开
 * @author: lkc
 */
bool Motor_Encoder_IsCompareEnabled(void)
{
    return s_encoder_compare_enabled;
}

/**
 * @description: 拷贝编码器采样累计诊断。
 * @detail: 只读，不改变校准或测速。diagnostics 为空则直接返回。
 * @param {motor_encoder_sampling_diagnostics_t *} diagnostics 输出，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Encoder_GetSamplingDiagnostics(motor_encoder_sampling_diagnostics_t *diagnostics)
{
    if (diagnostics != NULL) {
        diagnostics->fresh_frames = s_sampling_diagnostics.fresh_frames;
        diagnostics->max_sample_gap_us = s_sampling_diagnostics.max_sample_gap_us;
        diagnostics->stale_reseeds = s_sampling_diagnostics.stale_reseeds;
        diagnostics->max_sample_age_us = s_sampling_diagnostics.max_sample_age_us;
    }
}

/**
 * @description: 开关编码器对照采集。
 * @detail: 只改诊断开关。本函数不检查电机是否在转，也不改变控制反馈源；encoder compare 命令才在运行中拒绝。
 * @param {bool} enabled true=允许对照采集
 * @return {void} 无
 * @author: lkc
 */
void Motor_Encoder_SetCompareEnabled(bool enabled)
{
    s_encoder_compare_enabled = enabled;
}

/** 项目兼容初始化；独立复用请使用InitState显式传入配置。 */
void Motor_Encoder_RuntimeInit(motor_encoder_runtime_t *rt, uint32_t counts_per_rev, bool inverted)
{
    const motor_encoder_config_t config = {
        MOTOR_ENCODER_PLL_BW_RAD_S, MOTOR_ENCODER_MAX_RPM,
        MOTOR_ENCODER_JUMP_MARGIN_COUNTS, MOTOR_ENCODER_SAMPLE_TIMEOUT_S
    };
    (void)Motor_Encoder_InitState(rt, counts_per_rev == 0U ? 1U : counts_per_rev, inverted, &config);
}

/**
 * @description: 初始化 MT6701 绝对编码器与默认运行时实例
 * @detail: 初始化 I2C2 与 14 位角度运行时；首次成功采样前不建立有效零点
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_Init(void)
{
    Motor_MT6701_Init();
    Motor_Encoder_RuntimeInit(&s_encoder, Hal_Encoder_CountsPerRev(), MOTOR_ENCODER_INVERTED);
    s_encoder_sample_sequence = 0U;
    s_encoder_sample_time_us = 0U;
    s_encoder_have_sequence = false;
    printf("Encoder MT6701 %s: %u count/rev\r\n", Motor_MT6701_InterfaceName(),
           (unsigned)Hal_Encoder_CountsPerRev());
}

/**
 * @description: 读取最近一次 MT6701 绝对角度原始值
 * @return {uint16_t} 14 位角度计数，范围 [0, 16384)
 * @author: lkc
 */
uint16_t Motor_Encoder_GetCount(void)
{
    return Hal_Encoder_GetCount();
}

/**
 * @description: 清除默认实例的电角偏置与速度历史
 * @detail: MT6701 是绝对角度传感器，复位软件状态不会也不应改写传感器角度
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_Reset(void)
{
    Motor_Encoder_ClearZeroState(&s_encoder);
    s_encoder_sample_sequence = 0U;
    s_encoder_sample_time_us = 0U;
    s_encoder_have_sequence = false;
}

/**
 * @description: 用 HAL 当前计数与采样间隔更新默认实例转速
 * @param {float} dt_s  控制/采样周期 (s)
 * @return {*}
 * @author: lkc
 */
static void Motor_Encoder_UpdateWithTimeout(float dt_s, float timeout_s)
{
    motor_mt6701_status_t mt;
    uint32_t sequence;
    float sample_dt_s;
    float correction_dt_s;

    Motor_MT6701_GetStatus(&mt);
    sequence = mt.sample_sequence;
    Motor_Encoder_PredictWithTimeoutState(&s_encoder, dt_s, timeout_s);
    if (s_encoder.sample_age_s * 1.0e6f > (float)s_sampling_diagnostics.max_sample_age_us) {
        s_sampling_diagnostics.max_sample_age_us = (uint32_t)(s_encoder.sample_age_s * 1.0e6f);
    }
    if (mt.valid && (!s_encoder_have_sequence || sequence != s_encoder_sample_sequence)) {
        uint32_t gap_us = (uint32_t)(mt.sample_time_us - s_encoder_sample_time_us);
        s_sampling_diagnostics.fresh_frames++;
        if (s_encoder_have_sequence && gap_us > s_sampling_diagnostics.max_sample_gap_us) {
            s_sampling_diagnostics.max_sample_gap_us = gap_us;
        }
        if (s_encoder.sample_stale) {
            s_sampling_diagnostics.stale_reseeds++;
        }
        sample_dt_s = (float)(uint32_t)(mt.sample_time_us - s_encoder_sample_time_us) * 1.0e-6f;
        s_encoder_sample_sequence = sequence;
        s_encoder_sample_time_us = mt.sample_time_us;
        s_encoder_have_sequence = true;
        if (!s_encoder.have_last || s_encoder.pll_inverted != s_encoder.inverted || s_encoder.sample_stale ||
            sample_dt_s < MOTOR_ENCODER_DT_MIN_S || sample_dt_s > MOTOR_ENCODER_DT_MAX_S) {
            Motor_Encoder_SeedState(&s_encoder, Hal_Encoder_GetCount());
        } else {
            correction_dt_s = sample_dt_s;
            if (correction_dt_s > MOTOR_ENCODER_CORRECTION_DT_MAX_S) {
                correction_dt_s = MOTOR_ENCODER_CORRECTION_DT_MAX_S;
            }
            Motor_Encoder_CorrectSampleState(&s_encoder, Hal_Encoder_GetCount(), sample_dt_s, correction_dt_s);
        }
    }
}

/**
 * @description: 读取默认实例最新已验收的机械角（不使用 PLL 预测位置）
 * @return {float} 机械角度 (rad)，[0, 2π)
 * @author: lkc
 */
void Motor_Encoder_Update(float dt_s)
{
    Motor_Encoder_UpdateWithTimeout(dt_s, MOTOR_ENCODER_SAMPLE_TIMEOUT_S);
}

/**
 * @description: 按参考诊断周期更新编码器，不作为控制反馈。
 * @detail: 供 1 kHz 对照采集使用，采样超时按 5 ms。不得替代 Motor_Encoder_Update 的控制反馈。
 * @param {float} dt_s 本拍累计时间 (s)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Encoder_UpdateReference(float dt_s)
{
    /* 1kHz诊断采集允许数个周期调度余量；不用于控制反馈，仍声明过期。 */
    Motor_Encoder_UpdateWithTimeout(dt_s, 0.005f);
}

float Motor_Encoder_GetMechAngle(void)
{
    uint16_t count = s_encoder.have_last ? s_encoder.last_count : Hal_Encoder_GetCount();

    return Motor_Encoder_GetMechAngleState(&s_encoder, count);
}

/**
 * @description: 读取默认实例当前电角
 * @detail: 极对数取自 motor.ol_pole_pairs；小于 1 时由 GetElecAngleState 按 1 处理。绝对有效须先 Detect
 * @return {float} 电角度 (rad)，[0, 2π)
 * @author: lkc
 */
float Motor_Encoder_GetElecAngle(void)
{
    uint16_t count = s_encoder.have_last ? s_encoder.last_count : Hal_Encoder_GetCount();

    /* 极对数下限由 GetElecAngleState 统一保证。 */
    return Motor_Encoder_GetElecAngleState(&s_encoder, count, motor.ol_pole_pairs);
}

/**
 * @description: 读取默认实例已更新的机械转速
 * @return {float} 机械转速 (RPM)；无效时勿当有效零速
 * @author: lkc
 */
float Motor_Encoder_GetMechRpm(void)
{
    return Motor_Encoder_GetMechRpmState(&s_encoder);
}

/**
 * @description: 读取默认实例近零吸附前的机械转速。
 * @return {float} 原始机械转速 (RPM)
 * @author: lkc
 */
float Motor_Encoder_GetRawMechRpm(void)
{
    return Motor_Encoder_GetRawMechRpmState(&s_encoder);
}

/**
 * @description: 读取默认实例距最近一次计数变化的时间。
 * @return {float} 边沿年龄 (s)
 * @author: lkc
 */
float Motor_Encoder_GetEdgeAge(void)
{
    return Motor_Encoder_GetEdgeAgeState(&s_encoder);
}

/**
 * @description: 读取默认实例距最近一次 SSI/I2C 新帧的时间。
 * @return {float} 采样年龄 (s)
 * @author: lkc
 */
float Motor_Encoder_GetSampleAge(void)
{
    return s_encoder.sample_age_s;
}

/**
 * @description: 读取最近一次控制侧消费帧的时间戳。
 * @detail: 与 GetElecAngle/GetMechAngle 缓存角对应。零点或速度无效时不得当作有效参考。
 * @return {uint32_t} 时间戳 (us)
 * @author: lkc
 */
uint32_t Motor_Encoder_GetSampleTimeUs(void)
{
    return s_encoder_sample_time_us;
}

/**
 * @description: 查询默认实例是否处于信号故障。
 * @detail: MT6701 无效、采样超时或运行时计数突跳，任一成立即为故障。
 * @return {bool} true=信号不可用
 * @author: lkc
 */
bool Motor_Encoder_IsSignalFault(void)
{
    return !Motor_MT6701_IsValid() || s_encoder.sample_stale || Motor_Encoder_IsSignalFaultState(&s_encoder);
}

/**
 * @description: 读取默认实例累计被拒绝的异常样本数。
 * @return {uint32_t} 被拒绝样本数
 * @author: lkc
 */
uint32_t Motor_Encoder_GetInvalidCount(void)
{
    return Motor_Encoder_GetInvalidCountState(&s_encoder);
}

/**
 * @description: 读取默认实例每帧验收后的累计机械计数。
 * @return {uint32_t} 累计机械计数（模 2^32）
 * @author: lkc
 */
uint32_t Motor_Encoder_GetAccumulatedCount(void)
{
    return Motor_Encoder_GetAccumulatedCountState(&s_encoder);
}

/**
 * @description: 查询默认实例本拍转速是否有效
 * @return {bool} true=速度可信
 * @author: lkc
 */
bool Motor_Encoder_IsSpeedValid(void)
{
    return Motor_MT6701_IsValid() && Motor_Encoder_IsSpeedValidState(&s_encoder);
}

/**
 * @description: 查询默认实例电角零点是否已标定
 * @return {bool} true=绝对电角可用
 * @author: lkc
 */
bool Motor_Encoder_IsZeroValid(void)
{
    return Motor_Encoder_IsZeroValidState(&s_encoder);
}

/**
 * @description: 把已保存的编码器零点和方向装回运行时。
 * @param {float} offset_rad  电角零点 (rad)；valid 为 false 时忽略
 * @param {bool} inverted     true=机械正方向与计数方向相反
 * @param {bool} valid        true=零点可用
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_ApplyCalibration(float offset_rad, bool inverted, bool valid)
{
    Motor_Encoder_ApplyCalibrationState(&s_encoder, offset_rad, inverted, valid);
}

#ifdef __cplusplus
}
#endif
