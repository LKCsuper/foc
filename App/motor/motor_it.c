/*
 * @Description: FOC 电流采样中断
 *
 * 逻辑分析仪上的「一个周期 66.6us / 约 15kHz」是 PWM 周期，不是中断执行时间。
 * 中央对齐：f_pwm = f_tim / (2 * ARR)，ARR = SYSTEM_CORE_CLOCK / f_zv，
 * 所以开关频率是配置值 f_zv 的一半。f_zv = 30000 时周期正好约 66.7us；
 * 当前 MCCONF_FOC_F_ZV = 25000，周期约 80us（12.5kHz）。
 * VESC 用同一套定时器（见 Doc/VESC/vesc_foc.c 的 dt = 1/(f_zv/2)）。
 * VESC 里很小的那个数是中断执行时间（timer_time_now → m_last_adc_isr_duration，
 * 大约十几到 25us），两边比的不是同一个量。
 *
 * 执行时间看 foc_isr_cycles（DWT 周期数，168MHz 下除以 168 得到 us）。
 *
 * @Date: 2023-02-18 23:29:37
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
volatile uint32_t foc_isr_cycles;
volatile uint32_t foc_isr_cycles_v7;
volatile uint32_t foc_isr_cycles_max;
/* Private function prototypes -----------------------------------------------*/
static uint32_t foc_cycles_now(void);
/* Private functions ---------------------------------------------------------*/

/* Cortex-M4 cycle counter. Same role as VESC timer_time_now() around the FOC ISR. */
static uint32_t foc_cycles_now(void)
{
	if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		DWT->CYCCNT = 0U;
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	}
	return DWT->CYCCNT;
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
	uint32_t t_start = foc_cycles_now();
	(void)p;
	(void)flags;

	/*
	 * 中央对齐下 TIM1 的更新事件在波峰和波谷各来一次，ADC/DMA 因此每个 PWM
	 * 周期进两次中断。DIR=0（向上计数）对应 V7。HD75 是三电阻下桥采样，
	 * 没有 HW_HAS_PHASE_SHUNTS，V7 时下桥关断，电流无效。
	 * VESC（Doc/VESC/vesc_foc.c）在这里直接 return，只对 V0 做 FOC。
	 */
#ifndef HW_HAS_DUAL_MOTORS
#ifndef HW_HAS_PHASE_SHUNTS
	if ((TIM1->CR1 & TIM_CR1_DIR) == 0U) {
		foc_isr_cycles_v7 = foc_cycles_now() - t_start;
		return;
	}
#endif
#endif

	/* 创建电机结构体 */
	MOTOR_PARAM_S *stMotorNow = &motor;

	uint16_t curr1 = (uint16_t)GET_CURRENT1();
	uint16_t curr2 = (uint16_t)GET_CURRENT2();
	uint16_t curr3 = (uint16_t)GET_CURRENT3();
	float sens1 = ADC_VOLTS(ADC_IND_SENS1);
	float sens2 = ADC_VOLTS(ADC_IND_SENS2);
	float sens3 = ADC_VOLTS(ADC_IND_SENS3);

	stMotorNow->currentAdc[0] = curr1;
	stMotorNow->currentAdc[1] = curr2;
	stMotorNow->currentAdc[2] = curr3;

	stMotorNow->voltAdc[0] = sens1;
	stMotorNow->voltAdc[1] = sens2;
	stMotorNow->voltAdc[2] = sens3;

	/*
	采集电流,这里理论上应该减去偏差,减去偏差才对,
	偏移大约2045左右
	 */
	float ia = ((float)curr1 - 2045.0f) * FAC_CURRENT;
	float ib = ((float)curr2 - 2045.0f) * FAC_CURRENT;
	float ic = ((float)curr3 - 2045.0f) * FAC_CURRENT;

	float Ialpha = 0.0f;
	float Ibeta = 0.0f;
	float Id = 0.0f;
	float Iq = 0.0f;
	float va;
	float vb;
	float vc;

	/* 这里是电压偏移。分压比是 float 常量，避免整段变成 double。 */
	const float phase_scale = ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
	va = sens1 * phase_scale;
	vb = sens3 * phase_scale;
	vc = sens2 * phase_scale;

	/* 0.1f：裸 0.1 是 double，会把整条低通拖进软浮点。 */
	UTILS_LP_FAST(stMotorNow->v_bus, GET_INPUT_VOLTAGE(), 0.1f);

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

	{
		uint32_t elapsed = foc_cycles_now() - t_start;
		foc_isr_cycles = elapsed;
		if (elapsed > foc_isr_cycles_max) {
			foc_isr_cycles_max = elapsed;
		}
	}

	return;
}

#ifdef __cplusplus
}
#endif
