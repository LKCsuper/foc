/*
 * @Description: 扩展卡尔曼滤波无感观测器（αβ，四状态）
 * @Author: lkc
 *
 * 学习要点：
 *   1) 状态 x=[iα, iβ, ωe, θe]。用 PMSM 电压方程做预测，用实测电流做校正。θ 在状态里，正反转不必像反电势 atan2 那样切符号。
 *   2) 过程噪声 Q、量测噪声 R 决定信不信模型和信不信电流。不是 Ortega 磁链圆，也不是固定增益龙伯格。
 *   3) 静止/极低速 e≈0，(ω,θ) 不可观。本模块不含 PLL、开环接管。计算量明显大于磁链/龙伯格，默认不进 ISR。
 *   4) 默认不接入 Motor_Control_Step。与 motor_flux / motor_luenberger / motor_sensorless 并行，不改它们的行为。
 */
#ifndef MOTOR_EKF_H
#define MOTOR_EKF_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
/* Private define ------------------------------------------------------------*/
#define MOTOR_EKF_N (4)                        /* 状态维数：iα iβ ωe θe */
#define MOTOR_EKF_MIN_LINKAGE (1.0e-6f)        /* 额定磁链下限 (Wb) */
#define MOTOR_EKF_MIN_INDUCTANCE (1.0e-6f)     /* 电感下限 (H) */
#define MOTOR_EKF_MAX_SPEED (1500.0f)          /* |ωe| 上限 (rad/s) */
#define MOTOR_EKF_MIN_SPEED (50.0f)            /* |ωe| 可信下限 (rad/s)，约 240 RPM（2 对极） */
#define MOTOR_EKF_DEFAULT_Q_CURRENT (1.0e-3f)  /* 电流过程噪声默认 (A²) */
#define MOTOR_EKF_DEFAULT_Q_SPEED (80.0f)      /* 电角速度过程噪声默认 ((rad/s)²) */
#define MOTOR_EKF_DEFAULT_Q_ANGLE (1.0e-4f)    /* 电角过程噪声默认 (rad²) */
#define MOTOR_EKF_DEFAULT_R_CURRENT (5.0e-4f)  /* 电流量测噪声默认 (A²) */
/* Private typedef -----------------------------------------------------------*/
/**
 * @description: EKF 配置（不持久化；由调用者每拍传入）
 */
typedef struct {
    bool enabled;       /* true=本拍允许预测+校正 */
    float resistance;   /* 定子电阻 (Ω)，须 ≥0 */
    float inductance;   /* 定子电感 (H)，须 >0 */
    float flux_linkage; /* 额定永磁磁链 ψf (Wb) */
    float q_current;    /* 电流过程噪声 Qii (A²)；≤0 用默认 */
    float q_speed;      /* 电角速度过程噪声 ((rad/s)²)；≤0 用默认 */
    float q_angle;      /* 电角过程噪声 (rad²)；≤0 用默认 */
    float r_current;    /* 电流量测噪声 R (A²)；≤0 用默认 */
    float phase_offset; /* 固定相位补偿 (rad)，加在 θ 之后 */
} motor_ekf_config_t;

/**
 * @description: EKF 运行状态（不持久化）
 */
typedef struct {
    bool initialized;
    float x[MOTOR_EKF_N];                /* [iα, iβ, ωe, θe] */
    float p[MOTOR_EKF_N][MOTOR_EKF_N]; /* 协方差 */
    float angle;                         /* 输出电角 (rad)，含 phase_offset */
    float speed_e;                       /* 电角速度 (rad/s) */
} motor_ekf_t;

/**
 * @description: 本拍 EKF 输出
 */
typedef struct {
    bool valid;
    bool speed_ok;     /* |ωe| 是否高于 MOTOR_EKF_MIN_SPEED */
    float i_hat_alpha; /* 估算 α 轴电流 (A) */
    float i_hat_beta;  /* 估算 β 轴电流 (A) */
    float angle;       /* 电角 (rad) */
    float speed_e;     /* 电角速度 (rad/s) */
} motor_ekf_output_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 复位 EKF 状态
 * @param {motor_ekf_t} *obs  实例，可为 NULL
 * @return {*}
 * @author: lkc
 */
void Motor_Ekf_Reset(motor_ekf_t *obs);

/**
 * @description: 规整 EKF 配置
 * @detail: R 钳到 ≥0；L/ψf 低于下限时抬到下限；负的 Q/R 置 0，由 Step 套默认。
 * @param {motor_ekf_config_t} *config  配置，被原地修改；可为 NULL
 * @return {*}
 * @author: lkc
 */
void Motor_Ekf_NormalizeConfig(motor_ekf_config_t *config);

/**
 * @description: 用当前电流、假设电角和电角速度初始化状态与协方差
 * @param {motor_ekf_t} *obs                   实例，可为 NULL
 * @param {const motor_ekf_config_t} *config   配置；NULL 则忽略
 * @param {float} angle                        假设电角 (rad)
 * @param {float} omega_e                      假设电角速度 (rad/s)
 * @param {float} i_alpha                      定子 α 轴电流 (A)
 * @param {float} i_beta                       定子 β 轴电流 (A)
 * @return {*}
 * @author: lkc
 */
void Motor_Ekf_Seed(motor_ekf_t *obs, const motor_ekf_config_t *config, float angle, float omega_e, float i_alpha,
                    float i_beta);

/**
 * @description: 推进一拍扩展卡尔曼滤波
 * @detail: 预测 x=f(x,v)，P=FPFᵀ+Q；再用 iα/iβ 校正。S 奇异、NaN/Inf 或参数非法时复位并返回 false。
 * @param {motor_ekf_t} *obs                   实例
 * @param {const motor_ekf_config_t} *config   配置
 * @param {float} v_alpha                      定子 α 轴电压 (V)，通常为上一拍命令电压
 * @param {float} v_beta                       定子 β 轴电压 (V)
 * @param {float} i_alpha                      本拍 α 轴电流 (A)
 * @param {float} i_beta                       本拍 β 轴电流 (A)
 * @param {float} dt                           控制周期 (s)，须为有限正值
 * @param {motor_ekf_output_t} *output         本拍输出；不可为空
 * @return {bool} true=本拍观测有效
 * @author: lkc
 */
bool Motor_Ekf_Step(motor_ekf_t *obs, const motor_ekf_config_t *config, float v_alpha, float v_beta, float i_alpha,
                    float i_beta, float dt, motor_ekf_output_t *output);

/**
 * @description: 读取观测电角
 * @param {const motor_ekf_t} *obs  实例；NULL 返回 0
 * @return {float} 电角 (rad)
 * @author: lkc
 */
float Motor_Ekf_GetAngle(const motor_ekf_t *obs);

/**
 * @description: 读取估算电角速度
 * @param {const motor_ekf_t} *obs  实例；NULL 返回 0
 * @return {float} 电角速度 (rad/s)
 * @author: lkc
 */
float Motor_Ekf_GetSpeedE(const motor_ekf_t *obs);
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
