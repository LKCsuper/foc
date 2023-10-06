/*
 * @Description: foc中断,一个周期15k左右,66us
 * @Version: 1.0
 * @Author: lkc
 * @Date: 2023-10-01 10:28:50
 * @LastEditors: lkc
 * @LastEditTime: 2023-10-06 14:25:50
 * @detail: 
 * 
 * 开环注意：TIM1->ARR 为5600  modQ 为模拟vq
 * (1) modq 过大，就会使劲转一下，然后重启，所以此时应该增大角度，或者减小modq
 * (2) 角度过大，电机就会抖，可以增大modq或者减小角度
 * 
 * 
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
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

    float Ialpha = 0.0f;
    float Ibeta = 0.0f;
    float Id = 0.0f;
    float Iq = 0.0f;
    float va = 0.0f;
    float vb = 0.0f;
    float vc = 0.0f;
    float ia = 0.0f;
    float ib = 0.0f;
    float ic = 0.0f;
    float vBus = 0.0f;
    static float Theta = 0;

    ia = stMotorNow->ia;
    ib = stMotorNow->ib;
    ic = stMotorNow->ic;

    va = stMotorNow->va;
    vb = stMotorNow->vb;
    vc = stMotorNow->vc;

    vBus = stMotorNow->v_bus;

    float modD = 0;
#if 0
    float modQ = 0.4;
    Theta += 0.01;
#endif

#if 1
	  //float modD = 0;
    float modQ = 0.3;
    stMotorNow->theta += 0.001;
#endif
    //theta += 0.0001;
    if (stMotorNow->theta > (M_2PI))
    {
        stMotorNow->theta -= (M_2PI);
    }
#if 1
    rtU.Theta = stMotorNow->theta;
    rtU.Tpwm = 5600;
    rtU.Udc = stMotorNow->v_bus;
    rtU.Iq_ref = 1.0f;
    rtU.Id_ref = 0.0f;
    rtU.Ia = ia;
    rtU.Ib = ib;
    rtU.Ic = ic;
    foc_simulink0_step();
    TIMER_UPDATE_DUTY_M1(rtY.Tcm1, rtY.Tcm2, rtY.Tcm3);
    // PRINTF("{%f,%f,%f, %f}\r\n", rtU.Ia, rtU.Ib, rtU.Ic, TIM1->ARR);
    // PRINTF("{%f,%f,%f, %f}\r\n", rtY.Tcm1, rtY.Tcm2, rtY.Tcm3, TIM1->ARR);
    // PRINTF("{%f,%f}\r\n", rtY.Vd, rtY.Vq);
#else
    /* clarke 变换 */
    clarke_transform(ia, ib, ic, &Ialpha, &Ibeta);
    stMotorNow->Ialpha = Ialpha;
    stMotorNow->Ibeta = Ibeta;
    park_transform(Ialpha, Ibeta, stMotorNow->theta, &Id, &Iq);
    stMotorNow->Id = Id;
    stMotorNow->Iq = Iq;
    stMotorNow->e_iq_pid.Max_Output = 16.0f;
    stMotorNow->e_iq_pid.Min_Output = -16.0f;
    stMotorNow->e_iq_pid.B_Gain = 15.0f;
    stMotorNow->e_iq_pid.I_Gain = 450.0f;
    stMotorNow->e_iq_pid.P_Gain = 0.64167f  ;
    stMotorNow->e_iq_set = 0.5f;
    stMotorNow->e_id_set = 0.0f;
    Current_PID_Calc(stMotorNow->e_iq_set, stMotorNow->Iq, &stMotorNow->e_iq_out, &stMotorNow->e_iq_pid);
    stMotorNow->e_id_out = 0;
    float modAlpha = 0.0f;
    float modBeta = 0.0f;
    inverse_park(stMotorNow->e_id_out , stMotorNow->e_iq_out, stMotorNow->theta, &modAlpha, &modBeta);
    stMotorNow->modAlpha = modAlpha;
    stMotorNow->modBeta = modBeta;
    ULONG sector;
    svm_hqd(modAlpha, modBeta, vBus, TIM1->ARR, &stMotorNow->duty1, &stMotorNow->duty2, &stMotorNow->duty3, &sector);
    TIMER_UPDATE_DUTY_M1(stMotorNow->duty1, stMotorNow->duty2, stMotorNow->duty3);
#endif
	return;
}

#ifdef __cplusplus
}
#endif
