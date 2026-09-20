/*
 * @Description: 滑模反电势观测器（αβ 静止坐标系，饱和函数）
 * @Author: lkc
 *
 * 学习要点：
 *   1) PMSM：L·di/dt = v − R·i − e。滑模用电流误差做饱和开关 z，再拿 z 去修 ê。
 *   2) 不用 sign()：z = k·s/(|s|+ε)。离散 15 kHz 上 k>|e| 会让 dt·k/L>1，î 发散；所以电流方程只放 ê，z 不进电压差。
 *   3) s=î−i。s=0 时 z=0，ê 按 ω̂ 旋转即等价反电势。静止/极低速 e≈0，观测角不可信。不含 PLL、开环接管。
 *   4) 默认不接入 Motor_Control_Step。与 motor_flux / motor_luenberger / motor_ekf 并行，不改它们的行为。
 */
#ifndef MOTOR_SMO_H
#define MOTOR_SMO_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
/* Private define ------------------------------------------------------------*/
#define MOTOR_SMO_MIN_LINKAGE (1.0e-6f)        /* 额定磁链下限 (Wb) */
#define MOTOR_SMO_MIN_INDUCTANCE (1.0e-6f)     /* 电感下限 (H) */
#define MOTOR_SMO_MAX_GAIN (80.0f)             /* 滑模增益上限 (V) */
#define MOTOR_SMO_DEFAULT_GAIN (24.0f)         /* 默认 k (V)，只用来修 ê，不是电流方程里的满幅开关 */
#define MOTOR_SMO_DEFAULT_LPF (0.002f)         /* ê 对残差 z 的每拍掺入；约合龙伯格 l2·dt/k */
#define MOTOR_SMO_DEFAULT_EPS (0.08f)          /* 饱和边界 (A) */
#define MOTOR_SMO_MIN_BEMF (0.50f)             /* |ê| 可信下限 (V) */
/* Private typedef -----------------------------------------------------------*/
/**
 * @description: 滑模观测配置（不持久化；由调用者每拍传入）
 */
typedef struct {
    bool enabled;        /* true=本拍允许积分 */
    float resistance;    /* 定子电阻 (Ω)，须 ≥0 */
    float inductance;    /* 定子电感 (H)，须 >0 */
    float flux_linkage;  /* 额定永磁磁链 ψf (Wb)，用于 |ω|≈|e|/ψf */
    float slide_gain;    /* 滑模增益 k (V)；≤0 时用 24 V，只乘进 ê 校正 */
    float lpf;           /* ê 对 z 的每拍掺入 (0,1]；非法时用 0.002 */
    float sigmoid_eps;   /* 饱和边界 ε (A)；≤0 时用 0.08 */
    float phase_offset;  /* 固定相位补偿 (rad)，加在 atan2 之后 */
} motor_smo_config_t;

/**
 * @description: 滑模观测运行状态（不持久化）
 */
typedef struct {
    bool initialized;
    float i_hat_alpha; /* 估算 α 轴电流 (A) */
    float i_hat_beta;  /* 估算 β 轴电流 (A) */
    float z_alpha;     /* 滑模量 α (V)，s=0 时为 0；用来修 ê */
    float z_beta;      /* 滑模量 β (V) */
    float e_alpha;     /* 低通后反电势 α (V) */
    float e_beta;      /* 低通后反电势 β (V) */
    float magnitude;   /* |ê| (V) */
    float angle;       /* 原始观测电角 (rad)，约 [−π, π) */
    float speed_e;     /* 由 |ê|/ψf 带符号得到的电角速度 (rad/s) */
} motor_smo_t;

/**
 * @description: 本拍滑模观测输出
 */
typedef struct {
    bool valid;
    bool bemf_ok;    /* |ê| 是否高于 0.5 V */
    float e_alpha;   /* 反电势 α (V) */
    float e_beta;    /* 反电势 β (V) */
    float magnitude; /* |ê| (V) */
    float angle;     /* 原始观测电角 (rad) */
    float speed_e;   /* 电角速度 (rad/s) */
} motor_smo_output_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 复位滑模观测状态
 * @param {motor_smo_t} *obs  实例，可为 NULL
 * @return {*}
 * @author: lkc
 */
void Motor_Smo_Reset(motor_smo_t *obs);

/**
 * @description: 规整滑模观测配置
 * @detail: R 钳到 ≥0；L/ψf 低于下限时抬到下限；k 钳到 0～80 V；非法 lpf/ε 置 0，由 Step 套默认。
 * @param {motor_smo_config_t} *config  配置，被原地修改；可为 NULL
 * @return {*}
 * @author: lkc
 */
void Motor_Smo_NormalizeConfig(motor_smo_config_t *config);

/**
 * @description: 用当前电流、假设电角和电角速度初始化观测器
 * @detail: î = I，ê = ωe·ψf·[−sin θ, cos θ]，z = 0。ωe=0 时反电势从 0 开始。
 * @param {motor_smo_t} *obs                   实例，可为 NULL
 * @param {const motor_smo_config_t} *config   配置；NULL 则忽略
 * @param {float} angle                        假设电角 (rad)
 * @param {float} omega_e                      假设电角速度 (rad/s)
 * @param {float} i_alpha                      定子 α 轴电流 (A)
 * @param {float} i_beta                       定子 β 轴电流 (A)
 * @return {*}
 * @author: lkc
 */
void Motor_Smo_Seed(motor_smo_t *obs, const motor_smo_config_t *config, float angle, float omega_e, float i_alpha,
                    float i_beta);

/**
 * @description: 推进一拍滑模反电势观测
 * @detail: s=î−i；z=k·s/(|s|+ε)；L dî/dt = v−R·î−ê；ê 按 ω̂ 旋转后再掺入 lpf·z。
 *          未 Seed 时 î=I、ê=0。未使能、L 过小、dt 非法或 NaN/Inf 时复位并返回 false。
 * @param {motor_smo_t} *obs                   实例
 * @param {const motor_smo_config_t} *config   配置
 * @param {float} v_alpha                      定子 α 轴电压 (V)，通常为上一拍命令电压
 * @param {float} v_beta                       定子 β 轴电压 (V)
 * @param {float} i_alpha                      本拍 α 轴电流 (A)
 * @param {float} i_beta                       本拍 β 轴电流 (A)
 * @param {float} dt                           控制周期 (s)，须为有限正值
 * @param {motor_smo_output_t} *output         本拍输出；不可为空
 * @return {bool} true=本拍观测有效
 * @author: lkc
 */
bool Motor_Smo_Step(motor_smo_t *obs, const motor_smo_config_t *config, float v_alpha, float v_beta, float i_alpha,
                    float i_beta, float dt, motor_smo_output_t *output);

/**
 * @description: 读取原始观测电角
 * @param {const motor_smo_t} *obs  实例；NULL 返回 0
 * @return {float} 电角 (rad)
 * @author: lkc
 */
float Motor_Smo_GetAngle(const motor_smo_t *obs);

/**
 * @description: 读取估算反电势模长
 * @param {const motor_smo_t} *obs  实例；NULL 返回 0
 * @return {float} |ê| (V)
 * @author: lkc
 */
float Motor_Smo_GetMagnitude(const motor_smo_t *obs);

/**
 * @description: 读取估算电角速度
 * @param {const motor_smo_t} *obs  实例；NULL 返回 0
 * @return {float} 电角速度 (rad/s)
 * @author: lkc
 */
float Motor_Smo_GetSpeedE(const motor_smo_t *obs);
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
