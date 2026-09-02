/*
 * @Description:
 * @Date: 2023-02-18 23:29:45
 * @LastEditTime: 2023-02-19 17:13:24
 * @FilePath: \foc\App\motor\inc\utils.h
 */
#ifndef _MOTOR_PID_H
#define _MOTOR_PID_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 单轴电流 PI 控制器（d/q 轴通用，VESC 风格）
 * @param target:    目标电流 (A)
 * @param measured:  实测电流 (A)
 * @param v_int:     PI 积分项指针（调用者维护，传地址）
 * @param kp:        比例增益
 * @param ki:        积分增益
 * @param dt:        控制周期 (s)
 * @param v_limit:   输出电压限幅绝对值 (V)
 * @return:          输出电压 (V)，已限幅在 [-v_limit, +v_limit]
 */
float Motor_Current_PI_Run(float target, float measured, float *v_int,
                           float kp, float ki, float dt, float v_limit);

#ifdef __cplusplus
}
#endif

#endif
