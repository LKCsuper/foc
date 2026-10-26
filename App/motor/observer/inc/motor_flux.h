/*
 * @Description: Ortega 非线性磁链观测器（αβ 静止坐标系）
 * @Author: lkc
 *
 * 学习要点：
 *   1) PMSM：ψs = L×Is + ψr，v = R×I + dψs/dt。积分定子磁链再减 L×I 得到转子磁链，atan2 即电角。
 *   2) Ortega 用额定磁链圆约束 |ψr|，抑制纯积分漂移。本实现与 motor_sensorless 相同：只在 |ψr| 偏大时拉回，
 *      偏小时不主动放大，避免低速采样噪声把磁链撑爆。
 *   3) 静止/极低速反电势弱，观测角不可信。本模块只估磁链和原始电角，不含 PLL、开环接管或速度环。
 *   4) 默认不接入 Motor_Control_Step。现有无感路径仍用 motor_sensorless 内的同一公式，本文件不改其行为。
 */
#ifndef MOTOR_FLUX_H
#define MOTOR_FLUX_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
/* Private define ------------------------------------------------------------*/
#define MOTOR_FLUX_MIN_LINKAGE (1.0e-6f)     /* 额定磁链下限 (Wb)，防止除零 */
#define MOTOR_FLUX_GAIN_FALLBACK_NUM (1000.0f) /* 增益未配置时 γ = 1000 / ψf² */
#define MOTOR_FLUX_MAG_MIN_RATIO (0.50f)     /* |ψr| 相对额定磁链的可信下限 */
#define MOTOR_FLUX_MAG_MAX_RATIO (1.50f)     /* |ψr| 相对额定磁链的可信上限 */
/* Private typedef -----------------------------------------------------------*/
/**
 * @description: 磁链观测配置（不持久化；由调用者每拍传入）
 */
typedef struct {
    bool enabled;         /* true=本拍允许积分 */
    float resistance;     /* 定子电阻 (Ω)，须 ≥0 */
    float inductance;     /* 定子电感 (H)，须 ≥0 */
    float flux_linkage;   /* 额定永磁磁链 ψf (Wb) */
    float observer_gain;  /* Ortega 增益 γ；≤0 时按 1000/ψf² 回退 */
    float phase_offset;   /* 固定相位补偿 (rad)，加在 atan2 之后 */
} motor_flux_config_t;

/**
 * @description: 磁链观测运行状态（不持久化）
 */
typedef struct {
    bool initialized; /* 定子磁链积分器是否已 Seed */
    float x_alpha;    /* 估算定子磁链 α (Wb) */
    float x_beta;     /* 估算定子磁链 β (Wb) */
    float psi_alpha;  /* 估算转子磁链 α (Wb) */
    float psi_beta;   /* 估算转子磁链 β (Wb) */
    float magnitude;  /* |ψr| (Wb) */
    float angle;      /* 原始观测电角 (rad)，约 [−π, π) */
} motor_flux_t;

/**
 * @description: 本拍磁链观测输出
 */
typedef struct {
    bool valid;      /* true=本拍积分有限且已初始化 */
    bool flux_ok;    /* |ψr| 是否落在额定磁链的 50%~150% */
    float psi_alpha; /* 转子磁链 α (Wb) */
    float psi_beta;  /* 转子磁链 β (Wb) */
    float magnitude; /* |ψr| (Wb) */
    float angle;     /* 原始观测电角 (rad) */
} motor_flux_output_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 复位磁链观测状态（积分器与输出清零）
 * @param {motor_flux_t} *obs  实例，可为 NULL
 * @return {void} 无
 * @author: lkc
 */
void Motor_Flux_Reset(motor_flux_t *obs);

/**
 * @description: 规整磁链观测配置
 * @detail: R/L 钳到 ≥0；ψf 低于 1e-6 Wb 时抬到该下限；负增益置 0，由 Step 按 1000/ψf² 回退。
 * @param {motor_flux_config_t} *config  配置，被原地修改；可为 NULL
 * @return {void} 无
 * @author: lkc
 */
void Motor_Flux_NormalizeConfig(motor_flux_config_t *config);

/**
 * @description: 用当前电流和假设电角初始化定子磁链积分器
 * @detail: x = L×I + ψf·[cos θ, sin θ]。只是数值初值，不代表 θ 已是真转子角。
 * @param {motor_flux_t} *obs                   实例，可为 NULL
 * @param {const motor_flux_config_t} *config   配置；NULL 则忽略
 * @param {float} angle                         假设电角 (rad)
 * @param {float} i_alpha                       定子 α 轴电流 (A)
 * @param {float} i_beta                        定子 β 轴电流 (A)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Flux_Seed(motor_flux_t *obs, const motor_flux_config_t *config, float angle, float i_alpha, float i_beta);

/**
 * @description: 推进一拍 Ortega 磁链观测
 * @detail: dψs/dt = v − R×I + (γ/2)·ψr·min(0, ψf²−|ψr|²)；ψr = ψs − L×I；θ = atan2(ψrβ, ψrα)+offset。
 *          未 Seed 时按电角 0 自动初值。未使能、dt 非法或出现 NaN/Inf 时复位并返回 false。
 * @param {motor_flux_t} *obs                   实例
 * @param {const motor_flux_config_t} *config   配置
 * @param {float} v_alpha                       定子 α 轴电压 (V)，通常为上一拍命令电压
 * @param {float} v_beta                        定子 β 轴电压 (V)
 * @param {float} i_alpha                       本拍 α 轴电流 (A)
 * @param {float} i_beta                        本拍 β 轴电流 (A)
 * @param {float} dt                            控制周期 (s)，须为有限正值
 * @param {motor_flux_output_t} *output         本拍输出；不可为空
 * @return {bool} true=本拍观测有效
 * @author: lkc
 */
bool Motor_Flux_Step(motor_flux_t *obs, const motor_flux_config_t *config, float v_alpha, float v_beta, float i_alpha,
                     float i_beta, float dt, motor_flux_output_t *output);

/**
 * @description: 读取原始观测电角
 * @param {const motor_flux_t} *obs  实例；NULL 返回 0
 * @return {float} 电角 (rad)
 * @author: lkc
 */
float Motor_Flux_GetAngle(const motor_flux_t *obs);

/**
 * @description: 读取估算转子磁链模长
 * @param {const motor_flux_t} *obs  实例；NULL 返回 0
 * @return {float} |ψr| (Wb)
 * @author: lkc
 */
float Motor_Flux_GetMagnitude(const motor_flux_t *obs);
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
