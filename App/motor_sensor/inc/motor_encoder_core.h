/* 独立编码器算法接口，只依赖标准C；计数源可为绝对编码器或硬件AB计数器。 */
#ifndef MOTOR_ENCODER_CORE_H
#define MOTOR_ENCODER_CORE_H
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif
/* Private typedef -----------------------------------------------------------*/
typedef struct {
    float pll_bandwidth_rad_s; /* 正值，校正步长需满足2*bw*dt<1 */
    float max_rpm;             /* 机械转速能力，用于突跳拒绝与PLL限幅 */
    float jump_margin_counts;  /* 非负量化余量 count */
    float sample_timeout_s;    /* 新帧超时 s，正值 */
} motor_encoder_config_t;
/**
 * @description: 编码器运行时（ODrive PLL、零点偏置与速度有效标志）
 */
typedef struct {
    motor_encoder_config_t config; /* 实例持有配置，移植无需电机型号宏 */
    uint32_t counts_per_rev; /* 每机械圈计数（含倍频） */
    bool inverted;           /* true=方向取反 */
    float elec_offset_rad;   /* 电角度零点偏置 (rad) */
    bool zero_valid;         /* Detect/MarkZero 成功后为 true */

    uint16_t last_count; /* 上一拍原始计数 */
    uint32_t accumulated_count; /* 每帧验收后累计的有符号机械计数（模 2^32），供跨圈诊断 */
    bool have_last;      /* 是否已有上一拍（首拍速度无效） */
    bool pll_inverted;   /* pll_pos 采用的方向；与 inverted 不一致时重锚 */
    float pll_pos;       /* PLL 位置 (count)，范围 [0, cpr) */
    float pll_vel;       /* PLL 速度 (count/s)，近零已打成 0 */
    float edge_age_s;    /* 距最近一次计数变化的时间 (s) */
    float sample_age_s;  /* 距最近一次传感器新帧的时间 (s) */
    float mech_rpm_raw;  /* snap 前的机械转速 (RPM) */
    float mech_rpm;      /* snap 后的机械转速 (RPM) */
    bool speed_valid;    /* 本拍速度是否可信 */
    bool signal_fault;   /* 检出不可能的计数跳变 */
    bool sample_stale;   /* 传感器新帧超时 */
    uint32_t invalid_count; /* 累计拒绝的异常样本数 */
} motor_encoder_runtime_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 用新计数更新 ODrive PLL 的位置与转速
 * @detail: 不修改电角零点；dt 过小/过大或首拍时 speed_valid=false。PLL带宽由实例配置指定。
 * @param {motor_encoder_runtime_t} *rt  运行时状态
 * @param {uint16_t} count               当前正交解码计数值
 * @param {float} dt_s                   距上一拍时间 (s)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Encoder_UpdateState(motor_encoder_runtime_t *rt, uint16_t count, float dt_s);

/**
 * @description: 没有新传感器帧时只按 PLL 速度预测位置。
 * @detail: 采样年龄超过超时后清 speed_valid，不把缺帧当成零速。
 * @param {motor_encoder_runtime_t *} rt  运行时状态
 * @param {float} dt_s                    距上一拍时间 (s)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Encoder_PredictState(motor_encoder_runtime_t *rt, float dt_s);

/**
 * @description: 由原始计数计算单圈机械角（标定用，不含 PLL 插值）
 * @param {const motor_encoder_runtime_t} *rt  运行时状态
 * @param {uint16_t} count                     当前计数值
 * @return {float} 机械角度 (rad)，归一化到 [0, 2π)
 * @author: lkc
 */
float Motor_Encoder_GetMechAngleState(const motor_encoder_runtime_t *rt, uint16_t count);

/**
 * @description: 由运行时、当前计数与极对数计算电角度
 * @detail: elec = wrap(mech × pole_pairs − elec_offset)；绝对有效须 zero_valid=true
 * @param {const motor_encoder_runtime_t} *rt  运行时状态
 * @param {uint16_t} count                     当前计数值
 * @param {float} pole_pairs                   极对数；小于 1 时按 1 处理
 * @return {float} 电角度 (rad)，归一化到 [0, 2π)
 * @author: lkc
 */
float Motor_Encoder_GetElecAngleState(const motor_encoder_runtime_t *rt, uint16_t count, float pole_pairs);

/**
 * @description: 读取 UpdateState 后缓存的机械转速
 * @param {const motor_encoder_runtime_t} *rt  运行时状态
 * @return {float} 机械转速 (RPM)；speed_valid=false 时勿当作有效零速
 * @author: lkc
 */
float Motor_Encoder_GetMechRpmState(const motor_encoder_runtime_t *rt);

/**
 * @description: 读取近零吸附前的机械转速。
 * @param {const motor_encoder_runtime_t *} rt  运行时状态
 * @return {float} 原始机械转速 (RPM)；rt 为空返回 0
 * @author: lkc
 */
float Motor_Encoder_GetRawMechRpmState(const motor_encoder_runtime_t *rt);

/**
 * @description: 读取距最近一次计数变化的时间。
 * @param {const motor_encoder_runtime_t *} rt  运行时状态
 * @return {float} 边沿年龄 (s)；rt 为空返回 0
 * @author: lkc
 */
float Motor_Encoder_GetEdgeAgeState(const motor_encoder_runtime_t *rt);

/**
 * @description: 查询是否检测到不合理计数跳变。
 * @param {const motor_encoder_runtime_t *} rt  运行时状态
 * @return {bool} true=当前信号故障；rt 为空返回 false
 * @author: lkc
 */
bool Motor_Encoder_IsSignalFaultState(const motor_encoder_runtime_t *rt);

/**
 * @description: 读取累计被拒绝的异常样本数。
 * @param {const motor_encoder_runtime_t *} rt  运行时状态
 * @return {uint32_t} 被拒绝样本数；rt 为空返回 0
 * @author: lkc
 */
uint32_t Motor_Encoder_GetInvalidCountState(const motor_encoder_runtime_t *rt);

/**
 * @description: 读取每帧验收后累计的机械计数。
 * @detail: 正反方向已按 inverted 统一；返回值按 uint32_t 自然回绕，两次读数作 int32_t 差可得到短窗口增量。
 * @param {const motor_encoder_runtime_t *} rt  运行时状态
 * @return {uint32_t} 累计机械计数（模 2^32）；rt 为空返回 0
 * @author: lkc
 */
uint32_t Motor_Encoder_GetAccumulatedCountState(const motor_encoder_runtime_t *rt);

/**
 * @description: 查询本拍机械转速是否可信
 * @param {const motor_encoder_runtime_t} *rt  运行时状态
 * @return {bool} true=speed_valid；rt 为空返回 false
 * @author: lkc
 */
bool Motor_Encoder_IsSpeedValidState(const motor_encoder_runtime_t *rt);

/**
 * @description: 查询电角零点是否已标定（绝对电角可用）
 * @param {const motor_encoder_runtime_t} *rt  运行时状态
 * @return {bool} true=zero_valid；rt 为空返回 false
 * @author: lkc
 */
bool Motor_Encoder_IsZeroValidState(const motor_encoder_runtime_t *rt);

/**
 * @description: 将当前位置标为电角度 0（写入 offset 并置 zero_valid）
 * @param {motor_encoder_runtime_t} *rt  运行时状态
 * @param {uint16_t} count               当前计数值
 * @param {float} pole_pairs             极对数
 * @return {void} 无
 * @author: lkc
 */
void Motor_Encoder_MarkZeroState(motor_encoder_runtime_t *rt, uint16_t count, float pole_pairs);

/**
 * @description: 清除电角零点与 PLL 历史（Reset 路径用）
 * @param {motor_encoder_runtime_t} *rt  运行时状态
 * @return {void} 无
 * @author: lkc
 */
void Motor_Encoder_ClearZeroState(motor_encoder_runtime_t *rt);

/**
 * @description: 初始化独立状态，cpr为1..65536；非法输入返回false且不修改状态。
 * @param {motor_encoder_runtime_t} *rt  运行时状态，不可为空
 * @param {uint32_t} counts_per_rev      每机械圈计数（含正交倍频）
 * @param {bool} inverted                true=机械正方向与计数方向相反
 * @param {const motor_encoder_config_t *} config 配置
 * @return {bool} true=独立状态已初始化；false=输入非法且状态不修改
 */
bool Motor_Encoder_InitState(motor_encoder_runtime_t *rt, uint32_t counts_per_rev, bool inverted,
                             const motor_encoder_config_t *config);
/**
 * @description: 首帧/失联重建；保留电角零点和累计计数，本拍速度无效。rt不可为空。
 * @param {motor_encoder_runtime_t *} rt  运行时状态
 * @param {uint16_t} count               原始计数
 * @return {void} 无
 */
void Motor_Encoder_SeedState(motor_encoder_runtime_t *rt, uint16_t count);
/**
 * @description: 已初始化状态的新帧校正；两种dt单位s，由接入层保证有效且PLL稳定。
 * @param {motor_encoder_runtime_t *} rt  运行时状态
 * @param {uint16_t} count               新的原始计数
 * @param {float} sample_dt_s           距上一帧成功采样的时间 (s)，用于突跳门限
 * @param {float} correction_dt_s       PLL 校正步长 (s)，可小于 sample_dt_s
 * @return {void} 无
 */
void Motor_Encoder_CorrectSampleState(motor_encoder_runtime_t *rt, uint16_t count,
                                     float sample_dt_s, float correction_dt_s);
/**
 * @description: 无新帧预测；dt与timeout单位s。与默认PredictState共用同一算法。
 * @param {motor_encoder_runtime_t *} rt  运行时状态
 * @param {float} dt_s                    距上一拍时间 (s)
 * @param {float} timeout_s timeout_s，单位 s
 * @return {void} 无
 */
void Motor_Encoder_PredictWithTimeoutState(motor_encoder_runtime_t *rt, float dt_s, float timeout_s);
/**
 * @description: 应用外部校准，offset单位电角rad；保持测速历史，由后续更新处理方向变化。
 * @param {motor_encoder_runtime_t *} rt  运行时状态，不可为空
 * @param {float} offset_rad              电角零点偏置 (rad)
 * @param {bool} inverted                 true=方向取反
 * @param {bool} valid                    true=零点可用
 * @return {void} 无
 */
void Motor_Encoder_ApplyCalibrationState(motor_encoder_runtime_t *rt, float offset_rad, bool inverted, bool valid);
#ifdef __cplusplus
}
#endif
#endif
