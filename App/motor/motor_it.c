/*
 * @Description: foc中断,一个周期15k左右,66us
 * @Version: 1.0
 * @Author: lkc
 * @Date: 2023-10-01 10:28:50
 * @LastEditors: lkc
 * @LastEditTime: 2023-10-24 23:29:19
 * @detail: 
 * 
 * 开环注意：TIM1->ARR 为5600  modQ 为模拟vq
 * (1) modq 过大，就会使劲转一下，然后重启，所以此时应该增大角度，或者减小modq
 * (2) 角度过大，电机就会抖，可以增大modq或者减小角度
 * 
 * 电流环：
 * (1) 电流环的本质，你设置一个q值，并不是你实际输出的vq是符合这个q值，实际上是Iq，实际电机转换来的Iq最终符合这个设置q值
 * (2) 输入q值是正的，结果反馈是负的
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
static void Motor_OpenLoopSafeOutput(MOTOR_PARAM_S *stMotorNow)
{
    stMotorNow->duty1 = TIM1->ARR / 2;
    stMotorNow->duty2 = TIM1->ARR / 2;
    stMotorNow->duty3 = TIM1->ARR / 2;
    TIMER_UPDATE_DUTY_M1(stMotorNow->duty1, stMotorNow->duty2, stMotorNow->duty3);
    TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
}

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 中断中参数获取计算
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Motor_It_ParamCal(VOID)
{
    MOTOR_PARAM_S *stMotorNow = &motor;

	/* 获取三相adc值 */
	stMotorNow->currentAdc[0] = GET_CURRENT1();
	stMotorNow->currentAdc[1] = GET_CURRENT2();
	stMotorNow->currentAdc[2] = GET_CURRENT3();

	/* 计算三相电压,当前电阻的电压 */
	stMotorNow->voltAdc[0] = ADC_VOLTS(ADC_IND_SENS1);
	stMotorNow->voltAdc[1] = ADC_VOLTS(ADC_IND_SENS2);
	stMotorNow->voltAdc[2] = ADC_VOLTS(ADC_IND_SENS3);

	/* 
	采集电流,这里理论上应该减去偏差,减去偏差才对,
	偏移大约2045左右
	计算三相电流
	 */
    // stMotorNow->ia = (GET_CURRENT1() - stMotorNow->currentOffset[0]) * FAC_CURRENT;
	// stMotorNow->ib = (GET_CURRENT2() - stMotorNow->currentOffset[1]) * FAC_CURRENT;
	// stMotorNow->ic = (GET_CURRENT3() - stMotorNow->currentOffset[2]) * FAC_CURRENT;

	stMotorNow->ia = (GET_CURRENT1() - 2045) * FAC_CURRENT;
	stMotorNow->ib = (GET_CURRENT2() - 2045) * FAC_CURRENT;
	stMotorNow->ic = (GET_CURRENT3() - 2045) * FAC_CURRENT;

    stMotorNow->va = (ADC_VOLTS(ADC_IND_SENS1) - stMotorNow->voltundrivenOffset[0]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
	stMotorNow->vb = (ADC_VOLTS(ADC_IND_SENS3) - stMotorNow->voltundrivenOffset[2]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
	stMotorNow->vc = (ADC_VOLTS(ADC_IND_SENS2) - stMotorNow->voltundrivenOffset[1]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;

	/* 总线电压,根据电流计算 */
	UTILS_LP_FAST(stMotorNow->v_bus, GET_INPUT_VOLTAGE(), 0.1f);

	return;
}

// 1. 采集电流
// 2. clarke变换得到α β正交电量
// 3. park变换得到正交电流量 id iq 其中iq与转矩有关 id与磁通有关 实际控制经常把id置为0
// 4. pid控制
// 5. 通过角度得到电机转过的角度
// 6. 进行逆变换park变换
// 7. 对va vb 进行逆clarke变换,得到实际需要的三相电压
// TODO 停止pwm, 检测电流差异大约-0.3 到 0.3
void Motor_DmaInt(void *p, uint32_t flags)
{
    /* 目前电机频率是30khz，所以每两次运行一次双电机，所以15khz */
    static int skip = 0;
	if (++skip == FOC_CONTROL_LOOP_FREQ_DIVIDER) 
    {
		skip = 0;
	} 
    else 
    {
		return;
	}

    /* 创建电机结构体 */
    MOTOR_PARAM_S *stMotorNow = &motor; 
    /* 获取电流 电压参数 */
    Motor_It_ParamCal();

    /*
     * ============================================================
     *  开环 FOC（V/F 电压模式）
     *  - 只需设置 motor.ol_target_rpm（目标转速），即可运转
     *  - 角度自动递增，转速自动斜坡，Vq 由 V/F 曲线自动计算
     *  - 调参（shell 运行时修改）：
     *    ol_target_rpm  → 目标转速，电机应平稳转动（默认 500）
     *    ol_v_boost     → 电机卡/不转时增大（默认 2V）
     *    ol_kv_f        → 高速无力时增大（默认 0.05 V/Hz）
     *    ol_ramp_rpm_s  → 启动失步时减小（默认 500 RPM/s）
     *    ol_vq_limit    → 过流时减小（默认 18V）
     * ============================================================
     */
    float modAlpha = 0.0f;
    float modBeta  = 0.0f;
    float vBus = stMotorNow->v_bus;
    ULONG sector;

    if (vBus < 1.0f)
    {
        stMotorNow->ol_current_rpm = 0.0f;
        Motor_OpenLoopSafeOutput(stMotorNow);
        return;
    }

    float foc_freq   = stMotorNow->ol_foc_freq;    /* FOC 控制频率 Hz */
    float pole_pairs = stMotorNow->ol_pole_pairs;  /* 电机极对数 */
    float min_rpm    = stMotorNow->ol_min_rpm;     /* 最低转速地板 */

    if (!stMotorNow->ol_enable || stMotorNow->ol_target_rpm <= 0.0f)
    {
        stMotorNow->ol_target_rpm = 0.0f;
        stMotorNow->ol_current_rpm = 0.0f;
        Motor_OpenLoopSafeOutput(stMotorNow);
        return;
    }

    if (foc_freq <= 1.0f) foc_freq = 1.0f;
    if (pole_pairs < 1.0f) pole_pairs = 1.0f;
    if (min_rpm < 0.0f) min_rpm = 0.0f;
    if (stMotorNow->ol_ramp_rpm_s < 0.0f) stMotorNow->ol_ramp_rpm_s = 0.0f;
    if (stMotorNow->ol_v_boost < 0.0f) stMotorNow->ol_v_boost = 0.0f;
    if (stMotorNow->ol_kv_f < 0.0f) stMotorNow->ol_kv_f = 0.0f;
    if (stMotorNow->ol_vq_limit < 0.0f) stMotorNow->ol_vq_limit = 0.0f;
    if (stMotorNow->ol_max_rpm <= 0.0f) stMotorNow->ol_max_rpm = 1.0f;
    if (stMotorNow->ol_target_rpm > stMotorNow->ol_max_rpm) stMotorNow->ol_target_rpm = stMotorNow->ol_max_rpm;

    float vq_limit = stMotorNow->ol_vq_limit;
    if (vq_limit > vBus * 0.75f) vq_limit = vBus * 0.75f;

    /* ---- 软启动斜坡 ---- */
    {
        float dt = 1.0f / foc_freq;
        float step = stMotorNow->ol_ramp_rpm_s * dt;
        float target = stMotorNow->ol_target_rpm;

        /* 首次运行跳变到最低转速，避免从 0 起步导致抖而不转 */
        if (stMotorNow->ol_current_rpm < min_rpm && target > 0.0f)
        {
            stMotorNow->ol_current_rpm = min_rpm;
        }

        if (stMotorNow->ol_current_rpm < target)
        {
            stMotorNow->ol_current_rpm += step;
            if (stMotorNow->ol_current_rpm > target) stMotorNow->ol_current_rpm = target;
        }
        else if (stMotorNow->ol_current_rpm > target)
        {
            stMotorNow->ol_current_rpm -= step;
            if (stMotorNow->ol_current_rpm < target) stMotorNow->ol_current_rpm = target;
        }
    }

    /* ---- 角度递增（低于地板转速时，用地板值算角度，保证磁场一直转） ---- */
    {
        float rpm_for_angle = stMotorNow->ol_current_rpm;
        if (rpm_for_angle < min_rpm) rpm_for_angle = min_rpm;

        float elec_rad_s = RPM2RADPS_f(rpm_for_angle) * pole_pairs;
        stMotorNow->ol_theta += elec_rad_s / foc_freq;
        if (stMotorNow->ol_theta > M_2PI)  stMotorNow->ol_theta -= M_2PI;
        if (stMotorNow->ol_theta < 0.0f)   stMotorNow->ol_theta += M_2PI;
    }

    /* ---- V/F 曲线：Vq = V_boost + Kv_f × f_electrical（用实际转速） ---- */
    float vq;
    {
        float f_elec = stMotorNow->ol_current_rpm / 60.0f * pole_pairs;
        vq = stMotorNow->ol_v_boost + stMotorNow->ol_kv_f * f_elec;
        if (vq > vq_limit) vq = vq_limit;
    }

    inverse_park(0.0f, vq, stMotorNow->ol_theta, &modAlpha, &modBeta);
    svm_hqd(modAlpha, modBeta, vBus, TIM1->ARR,
            &stMotorNow->duty1, &stMotorNow->duty2, &stMotorNow->duty3, &sector);
    TIMER_UPDATE_DUTY_M1(stMotorNow->duty1, stMotorNow->duty2, stMotorNow->duty3);
    TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

	return;
}

#ifdef __cplusplus
}
#endif
