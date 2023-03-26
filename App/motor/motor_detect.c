/*
 * @Description: 检测参数
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2023-03-25 23:32:22
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

	osDelay(1000);
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

	osDelay(1);

	for (float i = 0;i < samples;i++) {
		current_sum[0] += motor.currentAdc[0];
		voltage_sum[0] += motor.voltAdc[0];
		osDelay(1);
	}

	// Start PWM on phase 2
	Bsp_Tim_StopPwm(MOTOR1);

	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, 	 TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, 	 TIM_Channel_2, TIM_CCxN_Enable);
	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

	osDelay(1);

	for (float i = 0;i < samples;i++) {
		current_sum[1] += motor.currentAdc[1];
		voltage_sum[1] += motor.voltAdc[1];
		osDelay(1);
	}

	// Start PWM on phase 3
	Bsp_Tim_StopPwm(MOTOR1);

	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, 	 TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, 	 TIM_Channel_3, TIM_CCxN_Enable);
	TIM_GenerateEvent(	 TIM1, TIM_EventSource_COM);

	osDelay(1);

	for (float i = 0;i < samples;i++) {
		current_sum[2] += motor.currentAdc[2];
		voltage_sum[2] += motor.voltAdc[2];
		osDelay(1);
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

	// TODO 为什么这里加打印,命令行调用和卡死串口
	// TODO 但是如果是在外设初始化完成后,这里有打印没问题
/*
	PRINTF("Current Offset [%f] [%f] [%f] \n", motor.currentOffset[0], motor.currentOffset[1], motor.currentOffset[2]);
	PRINTF("Volt Offset [%f] [%f] [%f] \n", motor.voltOffset[0], motor.voltOffset[1], motor.voltOffset[2]);
*/
#if 0
	// Measure undriven offsets
	// 有无负载
	if (cal_undriven) {
		osDelay(10);

		voltage_sum[0] = 0.0; voltage_sum[1] = 0.0; voltage_sum[2] = 0.0;

		for (float i = 0;i < samples;i++) {
			v_avg = (ADC_VOLTS(ADC_IND_SENS1) + ADC_VOLTS(ADC_IND_SENS2) + ADC_VOLTS(ADC_IND_SENS3)) / 3.0;
			voltage_sum[0] += ADC_VOLTS(ADC_IND_SENS1) - v_avg;
			voltage_sum[1] += ADC_VOLTS(ADC_IND_SENS2) - v_avg;
			voltage_sum[2] += ADC_VOLTS(ADC_IND_SENS3) - v_avg;
			osDelay(1);
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
#ifdef __cplusplus
}
#endif
