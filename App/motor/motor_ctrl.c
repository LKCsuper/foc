/*
 * @Description: 电机开环控制与 PWM 电压输出
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "motor_ctrl.h"
#include "motor_foc.h"
#include "motor_math.h"
#include "hal_motor.h"
/* Private define ------------------------------------------------------------*/
#define VOLTAGE_BUSTAR                                                      (24.0f)  /* 母线电压 */
float gfMechAngle = 0;              /* 机械角度 */
float gfOpenloopTimstamp = 0;       /* 开环时间戳 */        
float ualpha, ubeta;
float ua, ub, uc;
float ud, uq;
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void Motor_Ctrl_Pwm(float ua, float ub, float uc);
static void Motor_Ctrl_SetVoltage(float uq, float ud, float angle);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 将三相相电压转换为 PWM 占空比并写入定时器。
 * @detail: 输入为以中性点为 0 的双极性相电压 (V)，加 Vbus/2 偏置后归一化为占空比 [0, 1]。
 * @param {float} ua  A 相电压 (V)
 * @param {float} ub  B 相电压 (V)
 * @param {float} uc  C 相电压 (V)
 * @return {*}
 * @author: lkc
 */
static void Motor_Ctrl_Pwm(float ua, float ub, float uc)
{
    /* 偏置 Vbus/2：双极性相电压 -> 单极性 PWM 参考 [0, Vbus] */
    ua += 24;
    ub += 24;
    uc += 24;

    /* 限幅到 [0, 2*Vbus]，防止过调制 */
    utils_truncate_number(&ua, 0.0f, 2 * VOLTAGE_BUSTAR);
    utils_truncate_number(&ub, 0.0f, 2 * VOLTAGE_BUSTAR);
    utils_truncate_number(&uc, 0.0f, 2 * VOLTAGE_BUSTAR);

    /* 电压 -> 占空比 [0, 1] */
    float dutya = ua / (2 * VOLTAGE_BUSTAR);
    float dutyb = ub / (2 * VOLTAGE_BUSTAR);
    float dutyc = uc / (2 * VOLTAGE_BUSTAR);
    utils_truncate_number(&dutya, 0.0f, 1);
    utils_truncate_number(&dutyb, 0.0f, 1);
    utils_truncate_number(&dutyc, 0.0f, 1);

    /* 写入比较值并提交，三相占空比同步更新 */
    {
        uint32_t period = Hal_Motor_PwmPeriodCounts();

        Hal_Motor_PwmSetDuty((uint32_t)(period * dutya),
                             (uint32_t)(period * dutyb),
                             (uint32_t)(period * dutyc));
        Hal_Motor_PwmCommit();
    }
}

/**
 * @description: 按给定 dq 电压与电角度输出 PWM。
 * @detail: 经逆 Park → 逆 Clarke 得到三相相电压，再调用 Motor_Ctrl_Pwm 写入定时器。
 * @param {float} uq    q 轴电压 (V)
 * @param {float} ud    d 轴电压 (V)
 * @param {float} angle 电角度 (rad)
 * @return {*}
 * @author: lkc
 */
static void Motor_Ctrl_SetVoltage(float uq, float ud, float angle)
{
    utils_norm_angle_rad(&angle);
 
    /* 反park变换, */
	inverse_park(ud, uq, angle, &ualpha, &ubeta);

    /* clark逆变换 */
    inverse_clarke(ualpha, ubeta, &ua, &ub, &uc);

    Motor_Ctrl_Pwm(ua, ub, uc);
}

/**
 * @description: 简易开环拖动，按相邻两次调用的时间差对角度积分
 * @detail: 按 speed 与实测步长 Ts 累加机械角 gfMechAngle，归一化到弧度后
 *          施加固定 d/q 电压 (uq≈12V、ud=0)，经反 Park/Clarke 输出 PWM
 * @param {float} speed  开环转速指令，决定角度累加速率
 * @return {*}
 * @author: lkc
 */
void Motor_Ctrl_OpenLoop(float speed)
{
    uint32_t ulCurTimStamp = xTaskGetTickCount();
    static uint32_t ulOpenLoopTimStampe = 0;
    
        /* 计算每个loop时间间隔 */
    float Ts = (float)(ulCurTimStamp - ulOpenLoopTimStampe) * 1e-6f;

    if (0 == ulOpenLoopTimStampe)
    {
        Ts = 0;
    }

    /* 转换角度 */
	gfMechAngle += speed * Ts;
    gfMechAngle = DEG2RAD_f(gfMechAngle);
    printf("%f %f %lu %lu\r\n", gfMechAngle, Ts,
           (unsigned long)ulCurTimStamp, (unsigned long)ulOpenLoopTimStampe);

    float radian = gfMechAngle;

    /* 转换弧度 */
    utils_norm_angle_rad(&radian);

    
    /* 最高的电压 */
    float uq = 12.0;

    Motor_Ctrl_SetVoltage(uq, 0, utils_mech_to_electrical_angle(radian, 2));

    ulOpenLoopTimStampe = ulCurTimStamp;
}

/**
 * @description: 开环 SVPWM 占空比输出（定时器直通版）
 * @detail: 以 HAL 时基步长积分开环角，施加固定 uq=3V，经反 Park 后由
 *          foc_svm 直接生成三路 CCR 占空比写入定时器，不走电压偏置流程
 * @return {*}
 * @author: lkc
 */
void Motor_Ctrl_OpenLoop1(void)
{
    float radin = 0.0f;
    ud = 0;
    uq = 3.0;
     float sector;
    static uint32_t ulUa, ulUb, ulUc;
    static float m_openloop_angle = 0.0;
    uint32_t curTim = Hal_Motor_TimeUs();
    static uint32_t lastTim = 0;
    float dt = 0.0f;
    float phase = 0.0f;

    //radin = DEG2RAD_f(gfMechAngle);

    // * 开环实际需要的东西 时间差 开环角度
	// motor_now->m_openloop_angle += dt * motor_now->m_openloop_speed;
	// utils_norm_angle_rad((float*)&motor_now->m_openloop_angle);
	// motor_now->m_motor_state.phase = motor_now->m_openloop_angle;

    dt = Hal_Motor_TimeElapsedSec(curTim);

    if (0 == lastTim)
    {
        dt = 0;
    }

    m_openloop_angle += dt * 3000; // * 速度暂定3000
	utils_norm_angle_rad((float*)&m_openloop_angle);
	phase = m_openloop_angle;

    //utils_mech_to_electrical_angle(phase, 2);

#if 0
    //setPhaseVoltage(3.0f, 0.0f, _electricalAngle(radin, 2));
#endif

     inverse_park(ud, uq, phase, &ualpha, &ubeta);

     foc_svm(ualpha, ubeta, Hal_Motor_PwmPeriodCounts(), &ulUa, &ulUb, &ulUc , &sector);
    
//     svm(ualpha, ualpha, &ua, &ub, &uc);
    
    //gfMechAngle ++;
// //    // gfMechAngle = gfMechAngle % 360;
	
	// if (gfMechAngle - 360 > 0)
    // {
    //     gfMechAngle = 0;
    // }
    //vTaskDelay(20);
    //PRINTF("a%d b%d c %d %f %f %f %d\r\n", ulUa, ulUb, ulUc, gfMechAngle, phase, m_openloop_angle, lastTim);

    //Motor_Ctrl_Pwm(ua, ub, uc);
    /* 更新占空比 */
    Hal_Motor_PwmSetDuty(ulUa, ulUb, ulUc);
    Hal_Motor_PwmCommit();

    lastTim = curTim;
}
#ifdef __cplusplus
}
#endif
