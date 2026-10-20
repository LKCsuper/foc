/*
 * @Description: 弱磁：电压利用率触发负 Id（VESC duty 风格）
 * @Author: lkc
 *
 * 学习要点：
 *   1) 线性 SVPWM 电压上限约 Vbus/√3。高速时 ωeλ 会超过该上限，Id=0 无法再加速。
 *   2) 负 Id 削弱气隙磁链，降低反电势，把电压余量还给 Iq。
 *   3) 本模块只给出 Id_fw≤0，不改 PWM、不改 Iq；对齐阶段必须停用。
 *   4) 台架电源 3 A：弱磁与 Iq 共用电流圆，优先保留转矩电流。
 */
#ifndef MOTOR_FW_H
#define MOTOR_FW_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MOTOR_FW_SUPPLY_CURRENT (3.0f) /* 当前台架可调电源上限 (A) */
#define MOTOR_FW_I_BUDGET (2.5f)       /* 定子电流圆 (A)，留 0.5 A 给纹波和启动 */
/* Private typedef -----------------------------------------------------------*/
/**
 * @description: 弱磁运行状态（不持久化）
 */
typedef struct {
    float id;      /* 当前弱磁 Id，≤0 (A) */
    float last_vd; /* 上一拍已输出 Vd (V)，供本拍判断利用率 */
    float last_vq; /* 上一拍已输出 Vq (V) */
} motor_fw_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 复位弱磁状态（Id 与上一拍电压清零）
 * @param {motor_fw_t} *fw  弱磁实例，可为 NULL
 * @return {*}
 * @author: lkc
 */
void Motor_Fw_Reset(motor_fw_t *fw);

/**
 * @description: 在电流圆内给弱磁留出的 |Id| 余量
 * @detail: 优先保留 Iq；|Iq|≥i_budget 时返回 0。i_budget≤0 视为无余量。
 * @param {float} iq        本拍 Iq 目标 (A)
 * @param {float} i_budget  定子电流圆半径 (A)
 * @return {float} 允许的 |Id_fw| (A)，≥0
 * @author: lkc
 */
float Motor_Fw_IdRoom(float iq, float i_budget);

/**
 * @description: 按 |V|/Vlim 推进弱磁 Id
 * @detail: |V|/Vlim≥duty_start 时目标为 −current_max；低于 duty_start−0.05 时目标为 0；
 *          中间滞回保持。current_max≤0 视为关闭。Id 只允许 ≤0。
 * @param {motor_fw_t} *fw     弱磁实例
 * @param {float} current_max  |Id| 上限 (A)，≤0 关闭
 * @param {float} duty_start   开始弱磁的电压利用率，0.50~0.99
 * @param {float} ramp_a_s     Id 斜坡 (A/s)
 * @param {float} vd           用于判断的 D 轴电压 (V)，通常为上一拍输出
 * @param {float} vq           用于判断的 Q 轴电压 (V)
 * @param {float} v_limit      电压矢量上限 (V)
 * @param {float} dt           控制周期 (s)
 * @return {float} 本拍 Id_fw (A)，≤0
 * @author: lkc
 */
float Motor_Fw_Step(motor_fw_t *fw, float current_max, float duty_start, float ramp_a_s, float vd, float vq,
                    float v_limit, float dt);
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
