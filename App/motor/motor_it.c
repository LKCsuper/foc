/*
 * @Description: foc中断,一个周期15k左右,66us
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2023-03-26 10:47:17
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

	stMotorNow->currentAdc[0] = GET_CURRENT1();
	stMotorNow->currentAdc[1] = GET_CURRENT2();
	stMotorNow->currentAdc[2] = GET_CURRENT3();

	stMotorNow->voltAdc[0] = ADC_VOLTS(ADC_IND_SENS1);
	stMotorNow->voltAdc[1] = ADC_VOLTS(ADC_IND_SENS2);
	stMotorNow->voltAdc[2] = ADC_VOLTS(ADC_IND_SENS3);
	
	/* 
	采集电流,这里理论上应该减去偏差,减去偏差才对,
	偏移大约2045左右
	 */
	float ia = (GET_CURRENT1() - 2045) * FAC_CURRENT;
	float ib = (GET_CURRENT2() - 2045) * FAC_CURRENT;
	float ic = (GET_CURRENT3() - 2045) * FAC_CURRENT;
	
	float Ialpha = 0.0f;
	float Ibeta = 0.0f;
	float Id = 0.0f;
	float Iq = 0.0f;
	float va = 0.0f;
	float vb = 0.0f;
	float vc = 0.0f;

	/* 这里是电压偏移*/
	va = (ADC_VOLTS(ADC_IND_SENS1) - 0.0f) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
	vb = (ADC_VOLTS(ADC_IND_SENS3) - 0.0f) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
	vc = (ADC_VOLTS(ADC_IND_SENS2) - 0.0f) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
	
	/* 总线电压,根据电流计算 */
	UTILS_LP_FAST(stMotorNow->v_bus, GET_INPUT_VOLTAGE(), 0.1);

	/* 三相电流 */
	stMotorNow->ia = ia;
	stMotorNow->ib = ib;
	stMotorNow->ic = ic;

	stMotorNow->va = va;
	stMotorNow->vb = vb;
	stMotorNow->vc = vc;
	
/* 开环方案,每次改变实际位置 */
	static float theta = 0;

	// motor_now->m_motor_state.i_alpha = ia;
	// motor_now->m_motor_state.i_beta = ONE_BY_SQRT3 * ia + TWO_BY_SQRT3 * ib;

	// float vd_tmp = c * motor_now->m_motor_state.v_alpha + s * motor_now->m_motor_state.v_beta;
	// float vq_tmp = c * motor_now->m_motor_state.v_beta  - s * motor_now->m_motor_state.v_alpha;

	/*1. 给定d q值 */
	float modQ = 0.0f;
	float modD = 0.0f;


	/* park逆变换得到α β,角度用的是自己估测的角度 */
	inverse_park(modD, modQ, theta, &Ialpha, &Ibeta);
	stMotorNow->Ialpha = Ialpha;
	stMotorNow->Ibeta = Ibeta;


	/* 参考电压 */

	/* svpwm */
//	foc_svm(Ialpha, Ibeta, 0.5);

	return;
}

#ifdef __cplusplus
}
#endif
