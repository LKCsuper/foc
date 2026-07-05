/*
 * @Description: 检测参数
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2023-10-10 19:06:27
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
ULONG DETECT_Offset(BOOL cal_undriven)
{
	PRINTF("detect\n");

	osDelay(pdMS_TO_TICKS(1000));
	int cnt = 0;

	// TODO 看门狗时间必须延长 

	const float samples = 1000.0;
	float current_sum[3] = {0.0, 0.0, 0.0};
	float voltage_sum[3] = {0.0, 0.0, 0.0};

	/* 更新占空比 */
	TIMER_UPDATE_DUTY_M1(TIM1->ARR / 2, TIM1->ARR / 2, TIM1->ARR / 2);

	// Start PWM on phase 1
	Bsp_Tim_StopPwm(MOTOR1);

	// TODO 开启过滤
	//PHASE_FILTER_ON();
	// 下边很显然是开启pwm
	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, 	 TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, 	 TIM_Channel_1, TIM_CCxN_Enable);
	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

	osDelay(pdMS_TO_TICKS(1));

	for (float i = 0;i < samples;i++) {
		current_sum[0] += GET_CURRENT1();
		voltage_sum[0] += ADC_VOLTS(ADC_IND_SENS1);
		osDelay(pdMS_TO_TICKS(1));
	}

	// Start PWM on phase 2
	Bsp_Tim_StopPwm(MOTOR1);

	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, 	 TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, 	 TIM_Channel_2, TIM_CCxN_Enable);
	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

	osDelay(pdMS_TO_TICKS(1));

	for (float i = 0;i < samples;i++) {
		current_sum[1] += GET_CURRENT2();
		voltage_sum[1] += ADC_VOLTS(ADC_IND_SENS2);
		osDelay(pdMS_TO_TICKS(1));
	}

	// Start PWM on phase 3
	Bsp_Tim_StopPwm(MOTOR1);

	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, 	 TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, 	 TIM_Channel_3, TIM_CCxN_Enable);
	TIM_GenerateEvent(	 TIM1, TIM_EventSource_COM);

	osDelay(pdMS_TO_TICKS(1));

	for (float i = 0;i < samples;i++) {
		current_sum[2] += GET_CURRENT3();
		voltage_sum[2] += ADC_VOLTS(ADC_IND_SENS3);
		osDelay(pdMS_TO_TICKS(1));
	}

	Bsp_Tim_StopPwm(MOTOR1);

	motor.currentOffset[0] = current_sum[0] / samples;
	motor.currentOffset[1] = current_sum[1] / samples;
	motor.currentOffset[2] = current_sum[2] / samples;

	voltage_sum[0] /= samples;
	voltage_sum[1] /= samples;
	voltage_sum[2] /= samples;
	float v_avg = (voltage_sum[0] + voltage_sum[1] + voltage_sum[2]) / 3.0f;
	
	motor.voltOffset[0] = voltage_sum[0] - v_avg;
	motor.voltOffset[1] = voltage_sum[1] - v_avg;
	motor.voltOffset[2] = voltage_sum[2] - v_avg;

	// 为什么这里加打印,命令行调用和卡死串口
	// 但是如果是在外设初始化完成后,这里有打印没问题
	// !已解决，因为打印浮点数，这个任务栈不够用了
	PRINTF("ADC_RAW [%d] [%d] [%d]\r\n", (int)GET_CURRENT1(), (int)GET_CURRENT2(), (int)GET_CURRENT3());
	PRINTF("Current Offset [%f] [%f] [%f]\r\n", motor.currentOffset[0], motor.currentOffset[1], motor.currentOffset[2]);
	PRINTF("Volt Offset [%f] [%f] [%f]\r\n", motor.voltOffset[0], motor.voltOffset[1], motor.voltOffset[2]);

#if 1
	// Measure undriven offsets
	// 有无负载
	if (cal_undriven) {
		osDelay(pdMS_TO_TICKS(10));

		voltage_sum[0] = 0.0; voltage_sum[1] = 0.0; voltage_sum[2] = 0.0;

		for (float i = 0;i < samples;i++) {
			v_avg = (ADC_VOLTS(ADC_IND_SENS1) + ADC_VOLTS(ADC_IND_SENS2) + ADC_VOLTS(ADC_IND_SENS3)) / 3.0;
			voltage_sum[0] += ADC_VOLTS(ADC_IND_SENS1) - v_avg;
			voltage_sum[1] += ADC_VOLTS(ADC_IND_SENS2) - v_avg;
			voltage_sum[2] += ADC_VOLTS(ADC_IND_SENS3) - v_avg;
			osDelay(pdMS_TO_TICKS(1));
		}
		Bsp_Tim_StopPwm(MOTOR1);

		voltage_sum[0] /= samples;
		voltage_sum[1] /= samples;
		voltage_sum[2] /= samples;

		motor.voltundrivenOffset[0] = voltage_sum[0];
		motor.voltundrivenOffset[1] = voltage_sum[1];
		motor.voltundrivenOffset[2] = voltage_sum[2];
	}

	// TODO: Make sure that offsets are no more than e.g. 5%, as larger values indicate hardware problems.
	// TODO 看门狗时间恢复
#endif
	return 1;
}

#if 0
/* 检测电阻，原理其实就是电压除以电流 */
ULONG DETECT_resistance(float current, int samples, bool stop_after, float *resistance) {
	mc_interface_lock();

	volatile motor_all_state_t *motor = get_motor_now();
	int fault = FAULT_CODE_NONE;

	motor->m_phase_override = true;
	motor->m_phase_now_override = 0.0;
	motor->m_id_set = 0.0;
	motor->m_control_mode = CONTROL_MODE_CURRENT;
	motor->m_motor_released = false;
	motor->m_state = MC_STATE_RUNNING;

	// Disable timeout
	// systime_t tout = timeout_get_timeout_msec();
	// float tout_c = timeout_get_brake_current();
	// KILL_SW_MODE tout_ksw = timeout_get_kill_sw_mode();
	// timeout_reset();
	// timeout_configure(60000, 0.0, KILL_SW_MODE_DISABLED);

	// Ramp up the current slowly 慢慢升高电压
	while (fabsf(motor->m_iq_set - current) > 0.001) {
		utils_step_towards((float*)&motor->m_iq_set, current, fabsf(current) / 200.0);
		// fault = mc_interface_get_fault();
		// if (fault != FAULT_CODE_NONE) {
		// 	motor->m_id_set = 0.0;
		// 	motor->m_iq_set = 0.0;
		// 	motor->m_phase_override = false;
		// 	motor->m_control_mode = CONTROL_MODE_NONE;
		// 	motor->m_state = MC_STATE_OFF;
		// 	stop_pwm_hw((motor_all_state_t*)motor);

		// 	timeout_configure(tout, tout_c, tout_ksw);
		// 	mc_interface_unlock();

		// 	return fault;
		// }
		chThdSleepMilliseconds(1);
	}

	// Wait for the current to rise and the motor to lock.
	chThdSleepMilliseconds(50);

	// Sample
	motor->m_samples.avg_current_tot = 0.0;
	motor->m_samples.avg_voltage_tot = 0.0;
	motor->m_samples.sample_num = 0;

	int cnt = 0;
	while (motor->m_samples.sample_num < samples) {
		chThdSleepMilliseconds(1);
		cnt++;
		// Timeout
		if (cnt > 10000) {
			break;
		}
		fault = mc_interface_get_fault();
		if (fault != FAULT_CODE_NONE) {
			motor->m_id_set = 0.0;
			motor->m_iq_set = 0.0;
			motor->m_phase_override = false;
			motor->m_control_mode = CONTROL_MODE_NONE;
			motor->m_state = MC_STATE_OFF;
			stop_pwm_hw((motor_all_state_t*)motor);

			timeout_configure(tout, tout_c, tout_ksw);
			mc_interface_unlock();

			return fault;
		}
	}

	const float current_avg = motor->m_samples.avg_current_tot / (float)motor->m_samples.sample_num;
	const float voltage_avg = motor->m_samples.avg_voltage_tot / (float)motor->m_samples.sample_num;

	// Stop
	if (stop_after) {
		motor->m_id_set = 0.0;
		motor->m_iq_set = 0.0;
		motor->m_phase_override = false;
		motor->m_control_mode = CONTROL_MODE_NONE;
		motor->m_state = MC_STATE_OFF;
		stop_pwm_hw((motor_all_state_t*)motor);
	}

	// Enable timeout
	timeout_configure(tout, tout_c, tout_ksw);
	mc_interface_unlock();

	// 平均电流 / 平均电压
	*resistance = voltage_avg / current_avg;

	return fault;
}
#endif

/**
 * @description: 检测霍尔
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID DETECT_Hall(VOID)
{
	motor_all_state_t *motor = gpstMotor1;

	uint8_t hall_tab[8];
	bool res;
	mcpwm_foc_hall_detect(6.0 / 3.0, hall_tab, &res);

	/* 如果检测成功 */
	if (res)
	{
		INFO("DETECT HALL Success.\r\n");
		/* 获取霍尔table */
		for (int i = 0; i < 8; i++) {
			motor->m_conf->foc_hall_table[i] = hall_tab[i];
			PRINTF("%d ", hall_tab[i]);
		}
		PRINTF("\r\n");
		/* 获取霍尔table */
		for (int i = 0; i < 6; i++) {
			PRINTF("%d ", gpstMotor->hallABC[i]);
		}
		PRINTF("\r\n");
	}

	return;
}


/**
 * @description: 检测初始化
 * @detail: 
 * 注意如果先检测,adc检测中断中处理foc的函数一定要关闭
 * @return {*}
 * @author: lkc
 */
VOID Detect_Init(VOID)
{
	DETECT_Offset(TRUE);
	return;
}
#ifdef __cplusplus
}
#endif
