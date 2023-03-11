/*
 * @Description: 检测参数
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2023-03-11 11:32:01
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

/* 检测电流偏移*/
/**
 * Calibrate voltage and current offsets. For the observer to work at low modulation it
 * is very important to get all current and voltage offsets right. Therefore we store
 * the offsets for when the motor is undriven and when it is driven separately. The
 * motor is driven at 50% modulation on all phases when measuring the driven offset, which
 * corresponds to space-vector modulation with 0 amplitude.
 *校准电压和电流偏差。让观察者在低调制下工作
 *对于获得正确的电流和电压偏移非常重要。因此我们储存
 *电机不驱动和单独驱动时的偏移量。的
 *电机在测量驱动偏移时，在所有相位上以50%调制驱动
 *对应于振幅为0的空间矢量调制。
 * cal_undriven:   是否加负载
 * Calibrate undriven voltages too. This requires the motor to stand still.
 *
 * return:
 * -1: Timed out while waiting for fault code to go away.
 * 1: Success
 *
 */
int mcpwm_foc_dc_cal(bool cal_undriven) {
#if 0 
	// Wait max 5 seconds for DRV-fault to go away
	int cnt = 0;
	while(IS_DRV_FAULT()){
		chThdSleepMilliseconds(1);
		cnt++;
		if (cnt > 5000) {
			return -1;
		}
	};

	chThdSleepMilliseconds(1000);

	// Disable timeout
	systime_t tout = timeout_get_timeout_msec();
	float tout_c = timeout_get_brake_current();
	KILL_SW_MODE tout_ksw = timeout_get_kill_sw_mode();
	timeout_reset();
	timeout_configure(60000, 0.0, KILL_SW_MODE_DISABLED);

	// Measure driven offsets

	const float samples = 1000.0;
	float current_sum[3] = {0.0, 0.0, 0.0};
	float voltage_sum[3] = {0.0, 0.0, 0.0};

	TIMER_UPDATE_DUTY_M1(TIM1->ARR / 2, TIM1->ARR / 2, TIM1->ARR / 2);

	// Start PWM on phase 1
	stop_pwm_hw((motor_all_state_t*)&m_motor_1);
	PHASE_FILTER_ON();
	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

#ifdef HW_HAS_DUAL_MOTORS
	float current_sum_m2[3] = {0.0, 0.0, 0.0};
	float voltage_sum_m2[3] = {0.0, 0.0, 0.0};
	TIMER_UPDATE_DUTY_M2(TIM8->ARR / 2, TIM8->ARR / 2, TIM8->ARR / 2);

	stop_pwm_hw((motor_all_state_t*)&m_motor_2);
	PHASE_FILTER_ON_M2();
	TIM_SelectOCxM(TIM8, TIM_Channel_1, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM8, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM8, TIM_Channel_1, TIM_CCxN_Enable);
	TIM_GenerateEvent(TIM8, TIM_EventSource_COM);
#endif

	chThdSleep(1);

	for (float i = 0;i < samples;i++) {
		current_sum[0] += m_motor_1.m_currents_adc[0];
		voltage_sum[0] += ADC_VOLTS(ADC_IND_SENS1);
#ifdef HW_HAS_DUAL_MOTORS
		current_sum_m2[0] += m_motor_2.m_currents_adc[0];
		voltage_sum_m2[0] += ADC_VOLTS(ADC_IND_SENS4);
#endif
		chThdSleep(1);
	}

	// Start PWM on phase 2
	stop_pwm_hw((motor_all_state_t*)&m_motor_1);
	PHASE_FILTER_ON();
	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

#ifdef HW_HAS_DUAL_MOTORS
	stop_pwm_hw((motor_all_state_t*)&m_motor_2);
	PHASE_FILTER_ON_M2();
	TIM_SelectOCxM(TIM8, TIM_Channel_2, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM8, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM8, TIM_Channel_2, TIM_CCxN_Enable);
	TIM_GenerateEvent(TIM8, TIM_EventSource_COM);
#endif

	chThdSleep(1);

	for (float i = 0;i < samples;i++) {
		current_sum[1] += m_motor_1.m_currents_adc[1];
		voltage_sum[1] += ADC_VOLTS(ADC_IND_SENS2);
#ifdef HW_HAS_DUAL_MOTORS
		current_sum_m2[1] += m_motor_2.m_currents_adc[1];
		voltage_sum_m2[1] += ADC_VOLTS(ADC_IND_SENS5);
#endif
		chThdSleep(1);
	}

	// Start PWM on phase 3
	stop_pwm_hw((motor_all_state_t*)&m_motor_1);
	PHASE_FILTER_ON();
	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

#ifdef HW_HAS_DUAL_MOTORS
	stop_pwm_hw((motor_all_state_t*)&m_motor_2);
	PHASE_FILTER_ON_M2();
	TIM_SelectOCxM(TIM8, TIM_Channel_3, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM8, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM8, TIM_Channel_3, TIM_CCxN_Enable);
	TIM_GenerateEvent(TIM8, TIM_EventSource_COM);
#endif

	chThdSleep(1);

	for (float i = 0;i < samples;i++) {
		current_sum[2] += m_motor_1.m_currents_adc[2];
		voltage_sum[2] += ADC_VOLTS(ADC_IND_SENS3);
#ifdef HW_HAS_DUAL_MOTORS
		current_sum_m2[2] += m_motor_2.m_currents_adc[2];
		voltage_sum_m2[2] += ADC_VOLTS(ADC_IND_SENS6);
#endif
		chThdSleep(1);
	}

	stop_pwm_hw((motor_all_state_t*)&m_motor_1);

	m_motor_1.m_conf->foc_offsets_current[0] = current_sum[0] / samples;
	m_motor_1.m_conf->foc_offsets_current[1] = current_sum[1] / samples;
	m_motor_1.m_conf->foc_offsets_current[2] = current_sum[2] / samples;

	voltage_sum[0] /= samples;
	voltage_sum[1] /= samples;
	voltage_sum[2] /= samples;
	float v_avg = (voltage_sum[0] + voltage_sum[1] + voltage_sum[2]) / 3.0;

	m_motor_1.m_conf->foc_offsets_voltage[0] = voltage_sum[0] - v_avg;
	m_motor_1.m_conf->foc_offsets_voltage[1] = voltage_sum[1] - v_avg;
	m_motor_1.m_conf->foc_offsets_voltage[2] = voltage_sum[2] - v_avg;

#ifdef HW_HAS_DUAL_MOTORS
	stop_pwm_hw((motor_all_state_t*)&m_motor_2);

	m_motor_2.m_conf->foc_offsets_current[0] = current_sum_m2[0] / samples;
	m_motor_2.m_conf->foc_offsets_current[1] = current_sum_m2[1] / samples;
	m_motor_2.m_conf->foc_offsets_current[2] = current_sum_m2[2] / samples;

	voltage_sum_m2[0] /= samples;
	voltage_sum_m2[1] /= samples;
	voltage_sum_m2[2] /= samples;
	v_avg = (voltage_sum_m2[0] + voltage_sum_m2[1] + voltage_sum_m2[2]) / 3.0;

	m_motor_2.m_conf->foc_offsets_voltage[0] = voltage_sum_m2[0] - v_avg;
	m_motor_2.m_conf->foc_offsets_voltage[1] = voltage_sum_m2[1] - v_avg;
	m_motor_2.m_conf->foc_offsets_voltage[2] = voltage_sum_m2[2] - v_avg;
#endif

	// Measure undriven offsets

	if (cal_undriven) {
		chThdSleepMilliseconds(10);

		voltage_sum[0] = 0.0; voltage_sum[1] = 0.0; voltage_sum[2] = 0.0;
#ifdef HW_HAS_DUAL_MOTORS
		voltage_sum_m2[0] = 0.0; voltage_sum_m2[1] = 0.0; voltage_sum_m2[2] = 0.0;
#endif

		for (float i = 0;i < samples;i++) {
			v_avg = (ADC_VOLTS(ADC_IND_SENS1) + ADC_VOLTS(ADC_IND_SENS2) + ADC_VOLTS(ADC_IND_SENS3)) / 3.0;
			voltage_sum[0] += ADC_VOLTS(ADC_IND_SENS1) - v_avg;
			voltage_sum[1] += ADC_VOLTS(ADC_IND_SENS2) - v_avg;
			voltage_sum[2] += ADC_VOLTS(ADC_IND_SENS3) - v_avg;
#ifdef HW_HAS_DUAL_MOTORS
			v_avg = (ADC_VOLTS(ADC_IND_SENS4) + ADC_VOLTS(ADC_IND_SENS5) + ADC_VOLTS(ADC_IND_SENS6)) / 3.0;
			voltage_sum_m2[0] += ADC_VOLTS(ADC_IND_SENS4) - v_avg;
			voltage_sum_m2[1] += ADC_VOLTS(ADC_IND_SENS5) - v_avg;
			voltage_sum_m2[2] += ADC_VOLTS(ADC_IND_SENS6) - v_avg;
#endif
			chThdSleep(1);
		}

		stop_pwm_hw((motor_all_state_t*)&m_motor_1);

		voltage_sum[0] /= samples;
		voltage_sum[1] /= samples;
		voltage_sum[2] /= samples;

		m_motor_1.m_conf->foc_offsets_voltage_undriven[0] = voltage_sum[0];
		m_motor_1.m_conf->foc_offsets_voltage_undriven[1] = voltage_sum[1];
		m_motor_1.m_conf->foc_offsets_voltage_undriven[2] = voltage_sum[2];
#ifdef HW_HAS_DUAL_MOTORS
		stop_pwm_hw((motor_all_state_t*)&m_motor_2);

		voltage_sum_m2[0] /= samples;
		voltage_sum_m2[1] /= samples;
		voltage_sum_m2[2] /= samples;

		m_motor_2.m_conf->foc_offsets_voltage_undriven[0] = voltage_sum_m2[0];
		m_motor_2.m_conf->foc_offsets_voltage_undriven[1] = voltage_sum_m2[1];
		m_motor_2.m_conf->foc_offsets_voltage_undriven[2] = voltage_sum_m2[2];
#endif
	}

	// TODO: Make sure that offsets are no more than e.g. 5%, as larger values indicate hardware problems.

	// Enable timeout
	timeout_configure(tout, tout_c, tout_ksw);
	mc_interface_unlock();

	m_dccal_done = true;
#endif
	return 1;
}
#ifdef __cplusplus
}
#endif
