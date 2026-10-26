/*
 * @Description: 龙伯格反电势观测器（αβ 静止坐标系）
 * @Author: lkc
 *
 * 学习要点：
 *   1) PMSM：L·di/dt = v − R·i − e，e ≈ ωe·ψf·[−sin θ, cos θ]。电流误差去校正 ê，atan2(ê) 即电角。
 *   2) 经典龙伯格把 e 当缓变扰动；本实现再用上一拍 ω̂ 旋转 ê，减轻高速相位滞后。不是滑模，没有 Ortega 磁链圆。
 *   3) 静止/极低速 e≈0，观测角不可信。本模块不含 PLL、开环接管或速度环。
 *   4) 默认不接入 Motor_Control_Step。与 motor_flux / motor_sensorless 并行，不改它们的行为。
 */
#ifndef MOTOR_LUENBERGER_H
#define MOTOR_LUENBERGER_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
/* Private define ------------------------------------------------------------*/
#define MOTOR_LUENBERGER_MIN_LINKAGE (1.0e-6f)     /* 额定磁链下限 (Wb) */
#define MOTOR_LUENBERGER_MIN_INDUCTANCE (1.0e-6f)  /* 电感下限 (H)，防止除零 */
#define MOTOR_LUENBERGER_DEFAULT_WN (2500.0f)      /* 未配置增益时的观测带宽 (rad/s) */
#define MOTOR_LUENBERGER_DEFAULT_ZETA (0.707f)     /* 未配置增益时的阻尼比 */
#define MOTOR_LUENBERGER_MIN_BEMF (0.50f)          /* |ê| 可信下限 (V) */
/* Private typedef -----------------------------------------------------------*/
/**
 * @description: 龙伯格观测配置（不持久化；由调用者每拍传入）
 */
typedef struct {
    bool enabled;        /* true=本拍允许积分 */
    float resistance;    /* 定子电阻 (Ω)，须 ≥0 */
    float inductance;    /* 定子电感 (H)，须 >0 */
    float flux_linkage;  /* 额定永磁磁链 ψf (Wb)，用于 Seed 和 |ω|≈|e|/ψf */
    float current_gain;  /* 电流误差增益 l1 (1/s)；≤0 时按带宽自动整定 */
    float bemf_gain;     /* 反电势误差增益 l2 (V/(A·s))；≤0 时按带宽自动整定 */
    float phase_offset;  /* 固定相位补偿 (rad)，加在 atan2 之后 */
} motor_luenberger_config_t;

/**
 * @description: 龙伯格观测运行状态（不持久化）
 */
typedef struct {
    bool initialized;  /* 电流/反电势状态是否已 Seed */
    float i_hat_alpha; /* 估算 α 轴电流 (A) */
    float i_hat_beta;  /* 估算 β 轴电流 (A) */
    float e_alpha;     /* 估算 α 轴反电势 (V) */
    float e_beta;      /* 估算 β 轴反电势 (V) */
    float magnitude;   /* |ê| (V) */
    float angle;       /* 原始观测电角 (rad)，约 [−π, π) */
    float speed_e;     /* 由 |ê|/ψf 带符号得到的电角速度 (rad/s) */
} motor_luenberger_t;

/**
 * @description: 本拍龙伯格观测输出
 */
typedef struct {
    bool valid;      /* true=本拍积分有限且已初始化 */
    bool bemf_ok;    /* |ê| 是否高于 0.5 V */
    float e_alpha;   /* 反电势 α (V) */
    float e_beta;    /* 反电势 β (V) */
    float magnitude; /* |ê| (V) */
    float angle;     /* 原始观测电角 (rad) */
    float speed_e;   /* 电角速度 (rad/s) */
} motor_luenberger_output_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 复位龙伯格观测状态
 * @param {motor_luenberger_t} *obs  实例，可为 NULL
 * @return {void} 无
 * @author: lkc
 */
void Motor_Luenberger_Reset(motor_luenberger_t *obs);

/**
 * @description: 规整龙伯格观测配置
 * @detail: R 钳到 ≥0；L/ψf 低于下限时抬到下限；负增益置 0，由 Step 按默认带宽整定。
 * @param {motor_luenberger_config_t} *config  配置，被原地修改；可为 NULL
 * @return {void} 无
 * @author: lkc
 */
void Motor_Luenberger_NormalizeConfig(motor_luenberger_config_t *config);

/**
 * @description: 用当前电流、假设电角和电角速度初始化观测器
 * @detail: î = I，ê = ωe·ψf·[−sin θ, cos θ]。ωe=0 时反电势从 0 开始。
 * @param {motor_luenberger_t} *obs                  实例，可为 NULL
 * @param {const motor_luenberger_config_t} *config  配置；NULL 则忽略
 * @param {float} angle                              假设电角 (rad)
 * @param {float} omega_e                            假设电角速度 (rad/s)
 * @param {float} i_alpha                            定子 α 轴电流 (A)
 * @param {float} i_beta                             定子 β 轴电流 (A)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Luenberger_Seed(motor_luenberger_t *obs, const motor_luenberger_config_t *config, float angle, float omega_e,
                           float i_alpha, float i_beta);

/**
 * @description: 推进一拍龙伯格反电势观测
 * @detail: dî/dt = (v − R·î − ê)/L + l1(i−î)；dê/dt = ω̂×ê − l2(i−î)。
 *          未 Seed 时 î=I、ê=0。未使能、L 过小、dt 非法或 NaN/Inf 时复位并返回 false。
 * @param {motor_luenberger_t} *obs                  实例
 * @param {const motor_luenberger_config_t} *config  配置
 * @param {float} v_alpha                            定子 α 轴电压 (V)，通常为上一拍命令电压
 * @param {float} v_beta                             定子 β 轴电压 (V)
 * @param {float} i_alpha                            本拍 α 轴电流 (A)
 * @param {float} i_beta                             本拍 β 轴电流 (A)
 * @param {float} dt                                 控制周期 (s)，须为有限正值
 * @param {motor_luenberger_output_t} *output        本拍输出；不可为空
 * @return {bool} true=本拍观测有效
 * @author: lkc
 */
bool Motor_Luenberger_Step(motor_luenberger_t *obs, const motor_luenberger_config_t *config, float v_alpha,
                           float v_beta, float i_alpha, float i_beta, float dt, motor_luenberger_output_t *output);

/**
 * @description: 读取原始观测电角
 * @param {const motor_luenberger_t} *obs  实例；NULL 返回 0
 * @return {float} 电角 (rad)
 * @author: lkc
 */
float Motor_Luenberger_GetAngle(const motor_luenberger_t *obs);

/**
 * @description: 读取估算反电势模长
 * @param {const motor_luenberger_t} *obs  实例；NULL 返回 0
 * @return {float} |ê| (V)
 * @author: lkc
 */
float Motor_Luenberger_GetMagnitude(const motor_luenberger_t *obs);

/**
 * @description: 读取估算电角速度
 * @param {const motor_luenberger_t} *obs  实例；NULL 返回 0
 * @return {float} 电角速度 (rad/s)
 * @author: lkc
 */
float Motor_Luenberger_GetSpeedE(const motor_luenberger_t *obs);
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
