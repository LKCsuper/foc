/*
 * @Description: 编码器角度、速度与零点处理（当前硬件源为 MT6701）
 * @Author: lkc
 *
 * 学习要点：
 *   1) 电角绝对有效仅在零点标定（Detect）之后；上电仅清零计数不等于已标定。
 *   2) FOC 电角使用最新已验收的绝对计数；ODrive 临界阻尼 PLL 只估算转速。
 *   3) *State API 供主机双实例测试；默认包装读 HAL。绝对角查询仍用原始计数。
 */
#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/**
 * @description: 编码器运行时（ODrive PLL、零点偏置与速度有效标志）
 */
typedef struct {
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
 * @description: 初始化编码器运行时（不访问硬件）
 * @detail: 清零点与速度历史；counts_per_rev 为 0 时按 1 处理
 * @param {motor_encoder_runtime_t} *rt  运行时状态，不可为空
 * @param {uint32_t} counts_per_rev      每机械圈计数（含正交倍频）
 * @param {bool} inverted                true=机械正方向与计数方向相反
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_RuntimeInit(motor_encoder_runtime_t *rt, uint32_t counts_per_rev, bool inverted);

/**
 * @description: 用新计数更新 ODrive PLL 的位置与转速
 * @detail: 不修改电角零点；dt 过小/过大或首拍时 speed_valid=false。带宽约 100 rad/s。
 * @param {motor_encoder_runtime_t} *rt  运行时状态
 * @param {uint16_t} count               当前正交解码计数值
 * @param {float} dt_s                   距上一拍时间 (s)
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_UpdateState(motor_encoder_runtime_t *rt, uint16_t count, float dt_s);

/**
 * @description: 没有新传感器帧时只按 PLL 速度预测位置。
 * @detail: 采样年龄超过超时后清 speed_valid，不把缺帧当成零速。
 * @param {motor_encoder_runtime_t *} rt  运行时状态
 * @param {float} dt_s                    距上一拍时间 (s)
 * @return {*}
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
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_MarkZeroState(motor_encoder_runtime_t *rt, uint16_t count, float pole_pairs);

/**
 * @description: 清除电角零点与 PLL 历史（Reset 路径用）
 * @param {motor_encoder_runtime_t} *rt  运行时状态
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_ClearZeroState(motor_encoder_runtime_t *rt);

/**
 * @description: 初始化 AB 编码器硬件与默认运行时实例
 * @detail: 调用 Hal_Encoder_Init，按硬件每圈计数 RuntimeInit，并将计数清零
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_Init(void);

/**
 * @description: 读取正交解码当前计数值
 * @return {uint16_t} 当前计数值（硬件计数器宽度内）
 * @author: lkc
 */
uint16_t Motor_Encoder_GetCount(void);

/**
 * @description: 硬件计数清零，并清除默认实例的电角偏置与速度历史
 * @detail: 清零不等于已完成电角零点标定（zero_valid 会被清掉）
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_Reset(void);

/**
 * @description: 用 HAL 当前计数与采样间隔更新默认实例转速
 * @param {float} dt_s  控制/采样周期 (s)
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_Update(float dt_s);

/**
 * @description: 按参考诊断周期更新编码器，不作为控制反馈。
 * @detail: 供 1 kHz 对照采集使用，采样超时按 5 ms。不得替代 Motor_Encoder_Update 的控制反馈。
 * @param {float} dt_s 本拍累计时间 (s)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Encoder_UpdateReference(float dt_s);

/**
 * @description: 读取默认实例最新已验收的机械角（不使用 PLL 预测位置）
 * @return {float} 机械角度 (rad)，[0, 2π)
 * @author: lkc
 */
float Motor_Encoder_GetMechAngle(void);

/**
 * @description: 读取默认实例当前电角
 * @detail: 极对数取自全局 motor.ol_pole_pairs；绝对有效须先 Detect
 * @return {float} 电角度 (rad)，[0, 2π)
 * @author: lkc
 */
float Motor_Encoder_GetElecAngle(void);

/**
 * @description: 读取默认实例已更新的机械转速
 * @return {float} 机械转速 (RPM)；无效时勿当有效零速
 * @author: lkc
 */
float Motor_Encoder_GetMechRpm(void);

/**
 * @description: 读取默认实例近零吸附前的机械转速。
 * @return {float} 原始机械转速 (RPM)
 * @author: lkc
 */
float Motor_Encoder_GetRawMechRpm(void);

/**
 * @description: 读取默认实例距最近一次计数变化的时间。
 * @return {float} 边沿年龄 (s)
 * @author: lkc
 */
float Motor_Encoder_GetEdgeAge(void);

/**
 * @description: 读取默认实例距最近一次 SSI/I2C 新帧的时间。
 * @return {float} 采样年龄 (s)
 * @author: lkc
 */
float Motor_Encoder_GetSampleAge(void);

/**
 * @description: 读取最近一次控制侧消费帧的时间戳。
 * @detail: 与 GetElecAngle/GetMechAngle 缓存角对应。调用方应在短临界区里与角度一起捕获。
 *          零点或速度无效时不得当作有效参考。
 * @return {uint32_t} 时间戳 (us)
 * @author: lkc
 */
uint32_t Motor_Encoder_GetSampleTimeUs(void);

/**
 * @description: 查询默认实例是否处于信号故障。
 * @detail: MT6701 无效、采样超时或运行时计数突跳，任一成立即为故障。
 * @return {bool} true=信号不可用
 * @author: lkc
 */
bool Motor_Encoder_IsSignalFault(void);

/**
 * @description: 读取默认实例累计被拒绝的异常样本数。
 * @return {uint32_t} 被拒绝样本数
 * @author: lkc
 */
uint32_t Motor_Encoder_GetInvalidCount(void);

/**
 * @description: 读取默认实例每帧验收后的累计机械计数。
 * @return {uint32_t} 累计机械计数（模 2^32）
 * @author: lkc
 */
uint32_t Motor_Encoder_GetAccumulatedCount(void);

/**
 * @description: 查询默认实例本拍转速是否有效
 * @return {bool} true=速度可信
 * @author: lkc
 */
bool Motor_Encoder_IsSpeedValid(void);

/**
 * @description: 查询默认实例电角零点是否已标定
 * @return {bool} true=绝对电角可用
 * @author: lkc
 */
bool Motor_Encoder_IsZeroValid(void);

/**
 * @description: 无 Z 相时标定编码器方向和电角零点。
 * @detail: 阻塞在应用任务里调用。先在电角 0 爬升 Id，再扫过 π/2 判断计数方向；
 *          随后正反向各走一圈机械角，用角度差的正余弦均值求零点。成功后写入运行时并尝试保存 Flash。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Encoder_Detect(void);

/**
 * @description: 标定编码器方向和电角零点。
 * @detail: 阻塞在应用任务里调用，不在控制中断里跑。成功后写入运行时；
 *          save_to_flash 为 false 时不写 Flash。电机已运行、故障锁存或标定电流超出 (0, 0.5] A 时直接拒绝。
 * @param {bool} save_to_flash true=尝试保存，false=仅应用到 RAM
 * @return {void} 无
 * @author: lkc
 */
void Motor_Encoder_DetectWithSave(bool save_to_flash);

/**
 * @description: 开关编码器对照采集。
 * @detail: 只改诊断开关。本函数不检查电机是否在转，也不改变控制反馈源；encoder compare 命令才在运行中拒绝。
 * @param {bool} enabled true=允许对照采集
 * @return {void} 无
 * @author: lkc
 */
void Motor_Encoder_SetCompareEnabled(bool enabled);

/**
 * @description: 查询编码器对照采集是否打开。
 * @return {bool} true=对照采集已打开
 * @author: lkc
 */
bool Motor_Encoder_IsCompareEnabled(void);

/**
 * @description: 编码器采样只读累计诊断。
 * @detail: 不改变校准或测速。字段为异步近似快照，不是同一临界区的一致样本。
 */
typedef struct {
    uint32_t fresh_frames;      /* 已验收新帧数 */
    uint32_t max_sample_gap_us; /* 相邻新帧最大间隔 (us) */
    uint32_t stale_reseeds;     /* 采样过期后重新播种次数 */
    uint32_t max_sample_age_us; /* 观测到的最大采样年龄 (us) */
} motor_encoder_sampling_diagnostics_t;

/**
 * @description: 拷贝编码器采样累计诊断。
 * @detail: 只读，不改变校准或测速。diagnostics 为空则直接返回。
 * @param {motor_encoder_sampling_diagnostics_t *} diagnostics 输出，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Encoder_GetSamplingDiagnostics(motor_encoder_sampling_diagnostics_t *diagnostics);

/**
 * @description: 查询编码器电角零点对齐是否正在占用控制输出。
 * @return {bool} true=对齐进行中，本拍输出由检测目标决定
 * @author: lkc
 */
bool Motor_Encoder_DetectIsActive(void);

/**
 * @description: 读取编码器对齐阶段的固定电角和 Id 目标。
 * @param {float *} theta_rad   输出电角度 (rad)，不可为空
 * @param {float *} id_target   输出 Id 目标 (A)，不可为空
 * @param {float *} iq_target   输出 Iq 目标 (A)，不可为空；对齐时为 0
 * @return {bool} true=对齐进行中且三个指针都已写出
 * @author: lkc
 */
bool Motor_Encoder_DetectGetTargets(float *theta_rad, float *id_target, float *iq_target);

/**
 * @description: 把已保存的编码器零点和方向装回运行时。
 * @param {float} offset_rad  电角零点 (rad)；valid 为 false 时忽略
 * @param {bool} inverted     true=机械正方向与计数方向相反
 * @param {bool} valid        true=零点可用
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_ApplyCalibration(float offset_rad, bool inverted, bool valid);
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
