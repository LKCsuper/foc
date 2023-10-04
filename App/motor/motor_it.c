/*
 * @Description: foc中断,一个周期15k左右,66us
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2023-10-03 22:28:16
 * @FilePath: \foc\App\motor\utils.c
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
    /* 创建电机结构体 */
    MOTOR_PARAM_S *stMotorNow = &motor; 

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
    static float Theta = 0;

    ia = stMotorNow->ia;
    ib = stMotorNow->ib;
    ic = stMotorNow->ic;

    va = stMotorNow->va;
    vb = stMotorNow->vb;
    vc = stMotorNow->vc;

    float modD = 0;
#if 0
    float modQ = 0.4;
    Theta += 0.01;
#endif

#if 1
		float modQ = 0.3;
    Theta += 0.007;
#endif
    //theta += 0.0001;
    if (Theta > (M_2PI))
    {
    Theta -= (M_2PI);
    }

    rtU.Theta = Theta;
    rtU.Tpwm = 11200;
    rtU.Udc = 24.0;
    //rtU.Iq_ref = 0.3;
    rtU.Iq_ref = 0.5;
    rtU.Id_ref = 0;
    rtU.Ia = ia;
    rtU.Ib = ib;
    rtU.Ic = ic;


    /* clarke 变换 */
    clarke_transform(ia, ib, ic, &Ialpha, &Ibeta);
    stMotorNow->Ialpha = Ialpha;
    stMotorNow->Ibeta = Ibeta;
    stMotorNow->Theta = Theta;
    park_transform(Ialpha, Ibeta, Theta, &Id, &Iq);
    stMotorNow->Id = Id;
    stMotorNow->Iq = Iq;

    /* 位置环 */

    /* 速度环 */

    /* 电流环 */

    // float modD = stMotorNow->modD;
    // float modQ = stMotorNow->modQ;
    float modAlpha = 0.0f;
    float modBeta = 0.0f;
    inverse_park(modD, modQ, Theta, &modAlpha, &modBeta);
    stMotorNow->modAlpha = modAlpha;
    stMotorNow->modBeta = modBeta;


    ULONG duty1;
    ULONG duty2;
    ULONG duty3;
    ULONG sector;
    foc_svm(modAlpha, modBeta, TIM1->ARR, &duty1, &duty2, &duty3, &sector);
    //foc_simulink0_step();

    //PRINTF("{%d,%d,%d, %d}\r\n", duty1, duty2, duty3, TIM1->ARR);

    TIMER_UPDATE_DUTY_M1(duty1, duty2, duty3);
    //TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

	return;
}

#ifdef __cplusplus
}
#endif
