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
/*
 * 检测函数和 FOC 中断都会访问 TIM1。检测期间置位该标志，让 FOC 的 DMA
 * 回调立即返回，不运行保护、观测器、PI 或 SVPWM，避免双方抢写 TIM1。
 * ADC-DMA 搬运由硬件完成，不执行 FOC 回调也能持续更新 ADC_Value[]。
 */
static volatile bool s_detect_active = false;
static volatile bool s_detect_abort_requested = false;

#define MOTOR_DETECT_FLASH_KEY              "motor_detect_result"
#define MOTOR_DETECT_CURRENT_LIMIT_A        (4.0f)
#define MOTOR_DETECT_RESISTANCE_RAMP_S      (0.8f)
#define MOTOR_DETECT_RESISTANCE_SETTLE_S    (0.2f)
#define MOTOR_DETECT_RESISTANCE_SAMPLE_S    (0.5f)
#define MOTOR_DETECT_INDUCTANCE_ALIGN_S     (0.8f)
#define MOTOR_DETECT_INDUCTANCE_ZERO_S      (0.2f)
#define MOTOR_DETECT_INDUCTANCE_PULSE_V     (0.5f)
#define MOTOR_DETECT_INDUCTANCE_HALF_CYCLES (8U)
#define MOTOR_DETECT_INDUCTANCE_PULSES      (120U)
#define MOTOR_DETECT_FLUX_ALIGN_S           (1.0f)
#define MOTOR_DETECT_FLUX_RAMP_S            (4.0f)
#define MOTOR_DETECT_FLUX_SETTLE_S          (1.0f)
#define MOTOR_DETECT_FLUX_SAMPLE_S          (1.0f)
#define MOTOR_DETECT_VOLTAGE_LIMIT_V        (8.0f)

typedef enum {
	MOTOR_DETECT_STAGE_IDLE = 0,
	MOTOR_DETECT_STAGE_OFFSET,
	MOTOR_DETECT_STAGE_RESISTANCE,
	MOTOR_DETECT_STAGE_INDUCTANCE,
	MOTOR_DETECT_STAGE_FLUX,
} motor_detect_stage_t;

typedef struct {
	volatile motor_detect_stage_t stage;
	volatile bool done;
	volatile bool success;
	volatile motor_detect_error_t error;
	volatile mc_fault_code fault;
	float elapsed;
	float current_a;
	float electrical_rpm_target;
	float electrical_rpm_now;
	float theta;
	float vd;
	float vq;
	float sum_primary;
	float sum_secondary;
	float sum_tertiary;
	uint32_t samples;
	uint32_t pulse_position;
	uint32_t pulse_count;
	uint32_t valid_pulses;
	float pulse_start_current;
	bool pulse_started;
	bool rotation_started;
} motor_detect_control_t;

static motor_detect_control_t s_detect_control;
static motor_detect_result_t s_detect_result = {
	.version = MOTOR_DETECT_RESULT_VERSION,
};
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
	uint32_t sample_index;
	const uint32_t samples = 1000U;
	float current_sum[3] = {0.0f, 0.0f, 0.0f};
	float voltage_sum[3] = {0.0f, 0.0f, 0.0f};
	bool aborted = false;

	/*
	 * 第一步必须先置位互斥标志。标志置位后，下一次 DMA 中断只会清除
	 * 中断标志并返回，不再改写 PWM。然后撤销运行命令并关闭三相输出。
	 */
	s_detect_control.stage = MOTOR_DETECT_STAGE_OFFSET;
	s_detect_active = true;
	motor.ol_enable = false;
	motor.ol_target_rpm = 0.0f;
	motor.ol_current_rpm = 0.0f;
	motor.ol_vd_int = 0.0f;
	motor.ol_vq_int = 0.0f;
	Motor_SpeedLoop_Reset();
	Motor_Sensorless_Reset();
	Bsp_Tim_StopPwm(MOTOR1);
	PRINTF("Detect offset: begin\r\n");

	osDelay(pdMS_TO_TICKS(1000));

	// TODO 看门狗时间必须延长 

	/* 按 VESC 的偏移校准时序：50% 占空比后依次使能 A/B/C 相。 */
	TIMER_UPDATE_DUTY_M1(TIM1->ARR / 2, TIM1->ARR / 2, TIM1->ARR / 2);

	Bsp_Tim_StopPwm(MOTOR1);
	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
	osDelay(pdMS_TO_TICKS(1));
	PRINTF("Detect offset: phase A\r\n");
	for (sample_index = 0U; sample_index < samples; sample_index++) {
		if (s_detect_abort_requested) {
			aborted = true;
			break;
		}
		current_sum[0] += GET_CURRENT1();
		voltage_sum[0] += ADC_VOLTS(ADC_IND_SENS1);
		osDelay(pdMS_TO_TICKS(1));
	}
	if (aborted) goto detect_offset_finish;

	Bsp_Tim_StopPwm(MOTOR1);
	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
	osDelay(pdMS_TO_TICKS(1));
	PRINTF("Detect offset: phase B\r\n");
	for (sample_index = 0U; sample_index < samples; sample_index++) {
		if (s_detect_abort_requested) {
			aborted = true;
			break;
		}
		current_sum[1] += GET_CURRENT2();
		voltage_sum[1] += ADC_VOLTS(ADC_IND_SENS2);
		osDelay(pdMS_TO_TICKS(1));
	}
	if (aborted) goto detect_offset_finish;

	Bsp_Tim_StopPwm(MOTOR1);
	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
	osDelay(pdMS_TO_TICKS(1));
	PRINTF("Detect offset: phase C\r\n");
	for (sample_index = 0U; sample_index < samples; sample_index++) {
		if (s_detect_abort_requested) {
			aborted = true;
			break;
		}
		current_sum[2] += GET_CURRENT3();
		voltage_sum[2] += ADC_VOLTS(ADC_IND_SENS3);
		osDelay(pdMS_TO_TICKS(1));
	}
	if (aborted) goto detect_offset_finish;

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
		PRINTF("Detect offset: undriven voltage\r\n");
		osDelay(pdMS_TO_TICKS(10));

		voltage_sum[0] = 0.0; voltage_sum[1] = 0.0; voltage_sum[2] = 0.0;

		for (sample_index = 0U; sample_index < samples; sample_index++) {
			if (s_detect_abort_requested) {
				aborted = true;
				break;
			}
			v_avg = (ADC_VOLTS(ADC_IND_SENS1) + ADC_VOLTS(ADC_IND_SENS2) + ADC_VOLTS(ADC_IND_SENS3)) / 3.0;
			voltage_sum[0] += ADC_VOLTS(ADC_IND_SENS1) - v_avg;
			voltage_sum[1] += ADC_VOLTS(ADC_IND_SENS2) - v_avg;
			voltage_sum[2] += ADC_VOLTS(ADC_IND_SENS3) - v_avg;
			osDelay(pdMS_TO_TICKS(1));
		}
		if (aborted) goto detect_offset_finish;
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
	/*
	 * 先确认三相输出已经关闭，再释放互斥标志。检测后保持停机，避免
	 * 使用刚测完的参数自动起转；后续必须由 olspeed/olstart 明确启动。
	 */
detect_offset_finish:
	Bsp_Tim_StopPwm(MOTOR1);
	s_detect_control.stage = MOTOR_DETECT_STAGE_IDLE;
	s_detect_active = false;
	PRINTF(aborted ? "Detect offset: aborted\r\n" : "Detect offset: done\r\n");
	return aborted ? 0U : 1U;
}

bool Motor_Detect_IsActive(void)
{
	return s_detect_active;
}

bool Motor_Detect_ControlIsActive(void)
{
	return s_detect_active &&
		(s_detect_control.stage == MOTOR_DETECT_STAGE_RESISTANCE ||
		 s_detect_control.stage == MOTOR_DETECT_STAGE_INDUCTANCE ||
		 s_detect_control.stage == MOTOR_DETECT_STAGE_FLUX);
}

static float Motor_Detect_Clamp(float value, float minimum, float maximum)
{
	if (value > maximum) {
		return maximum;
	}
	if (value < minimum) {
		return minimum;
	}
	return value;
}

static float Motor_Detect_PhaseCurrentPeak(const MOTOR_PARAM_S *motor)
{
	float peak = fabsf(motor->ia);

	if (fabsf(motor->ib) > peak) peak = fabsf(motor->ib);
	if (fabsf(motor->ic) > peak) peak = fabsf(motor->ic);
	return peak;
}

static void Motor_Detect_FinishFromIsr(bool success, motor_detect_error_t error)
{
	Motor_DetectControl_OutputSafe(&motor);
	s_detect_control.success = success;
	s_detect_control.error = error;
	s_detect_control.done = true;
	s_detect_control.stage = MOTOR_DETECT_STAGE_IDLE;
	s_detect_active = false;
}

void Motor_Detect_AbortFromIsr(mc_fault_code fault)
{
	s_detect_control.fault = fault;
	Motor_Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_FAULT);
}

static void Motor_Detect_RunResistanceStep(MOTOR_PARAM_S *motor, float dt)
{
	float ramp;
	float id_target;
	float current_magnitude;
	float voltage_magnitude;
	float total_time = MOTOR_DETECT_RESISTANCE_RAMP_S +
		MOTOR_DETECT_RESISTANCE_SETTLE_S +
		MOTOR_DETECT_RESISTANCE_SAMPLE_S;

	s_detect_control.elapsed += dt;
	ramp = Motor_Detect_Clamp(
		s_detect_control.elapsed / MOTOR_DETECT_RESISTANCE_RAMP_S, 0.0f, 1.0f);
	id_target = s_detect_control.current_a * ramp;

	Motor_DetectControl_RunCurrent(motor, 0.0f, id_target, 0.0f,
		3.0f, dt, &s_detect_control.vd, &s_detect_control.vq);

	if (Motor_Detect_PhaseCurrentPeak(motor) > MOTOR_DETECT_CURRENT_LIMIT_A) {
		Motor_Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_CURRENT);
		return;
	}

	if (s_detect_control.elapsed >
		(MOTOR_DETECT_RESISTANCE_RAMP_S + MOTOR_DETECT_RESISTANCE_SETTLE_S)) {
		current_magnitude = NORM2_f(motor->Id, motor->Iq);
		voltage_magnitude = NORM2_f(s_detect_control.vd, s_detect_control.vq);
		if (current_magnitude > s_detect_control.current_a * 0.25f) {
			s_detect_control.sum_primary += voltage_magnitude;
			s_detect_control.sum_secondary += current_magnitude;
			s_detect_control.samples++;
		}
	}

	if (s_detect_control.elapsed >= total_time) {
		float resistance;

		if (s_detect_control.samples < 100U ||
			s_detect_control.sum_secondary <= 0.0f) {
			Motor_Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_RESISTANCE);
			return;
		}

		resistance = s_detect_control.sum_primary /
			s_detect_control.sum_secondary;
		if (resistance < 0.005f || resistance > 10.0f) {
			Motor_Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_RESISTANCE);
			return;
		}

		s_detect_result.resistance_ohm = resistance;
		s_detect_result.resistance_valid = true;
		Motor_Detect_FinishFromIsr(true, MOTOR_DETECT_ERROR_NONE);
	}
}

static void Motor_Detect_RunInductanceStep(MOTOR_PARAM_S *motor, float dt)
{
	float pulse_time_start = MOTOR_DETECT_INDUCTANCE_ALIGN_S +
		MOTOR_DETECT_INDUCTANCE_ZERO_S;

	s_detect_control.elapsed += dt;

	if (s_detect_control.elapsed < MOTOR_DETECT_INDUCTANCE_ALIGN_S) {
		float ramp = Motor_Detect_Clamp(
			s_detect_control.elapsed / MOTOR_DETECT_INDUCTANCE_ALIGN_S, 0.0f, 1.0f);
		Motor_DetectControl_RunCurrent(motor, 0.0f,
			s_detect_control.current_a * ramp, 0.0f,
			3.0f, dt, NULL, NULL);
		return;
	}

	if (s_detect_control.elapsed < pulse_time_start) {
		Motor_DetectControl_RunCurrent(motor, 0.0f, 0.0f, 0.0f,
			2.0f, dt, NULL, NULL);
		return;
	}

	if (!s_detect_control.pulse_started) {
		Motor_DetectControl_Reset(motor);
		s_detect_control.pulse_started = true;
		s_detect_control.pulse_position = 0U;
		s_detect_control.pulse_count = 0U;
	}

	{
		float sign = (s_detect_control.pulse_count & 1U) ? -1.0f : 1.0f;
		float voltage = sign * MOTOR_DETECT_INDUCTANCE_PULSE_V;

		Motor_DetectControl_RunVoltage(motor, 0.0f, voltage, 0.0f,
			MOTOR_DETECT_INDUCTANCE_PULSE_V);

		if (s_detect_control.pulse_position == 0U) {
			s_detect_control.pulse_start_current = motor->Id;
		}

		if (s_detect_control.pulse_position >=
			(MOTOR_DETECT_INDUCTANCE_HALF_CYCLES - 1U)) {
			float current_end = motor->Id;
			float current_mid = 0.5f *
				(s_detect_control.pulse_start_current + current_end);
			float delta_current = current_end -
				s_detect_control.pulse_start_current;
			float slope_dt = (float)(MOTOR_DETECT_INDUCTANCE_HALF_CYCLES - 1U) * dt;
			float slope = delta_current / slope_dt;
			float effective_voltage = voltage -
				s_detect_result.resistance_ohm * current_mid;

			if (fabsf(slope) > 1.0f) {
				float inductance = effective_voltage / slope;
				if (inductance > 5.0e-6f && inductance < 0.05f) {
					s_detect_control.sum_primary += inductance;
					s_detect_control.valid_pulses++;
				}
			}

			s_detect_control.pulse_count++;
			s_detect_control.pulse_position = 0U;
		} else {
			s_detect_control.pulse_position++;
		}
	}

	if (Motor_Detect_PhaseCurrentPeak(motor) > MOTOR_DETECT_CURRENT_LIMIT_A) {
		Motor_Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_CURRENT);
		return;
	}

	if (s_detect_control.valid_pulses >= MOTOR_DETECT_INDUCTANCE_PULSES ||
		s_detect_control.pulse_count >= MOTOR_DETECT_INDUCTANCE_PULSES * 3U) {
		float inductance;

		if (s_detect_control.valid_pulses < MOTOR_DETECT_INDUCTANCE_PULSES / 2U) {
			Motor_Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_INDUCTANCE);
			return;
		}

		/* 与 VESC 一样略微低估 L，观测器对低估通常比高估更稳定。 */
		inductance = 0.9f * s_detect_control.sum_primary /
			(float)s_detect_control.valid_pulses;
		s_detect_result.inductance_h = inductance;
		s_detect_result.inductance_valid = true;
		Motor_Detect_FinishFromIsr(true, MOTOR_DETECT_ERROR_NONE);
	}
}

static void Motor_Detect_RunFluxStep(MOTOR_PARAM_S *motor, float dt)
{
	float rotation_time;
	float transition;
	float speed_ramp;
	float id_target;
	float iq_target;
	float omega_electrical;
	float total_rotation_time = MOTOR_DETECT_FLUX_RAMP_S +
		MOTOR_DETECT_FLUX_SETTLE_S + MOTOR_DETECT_FLUX_SAMPLE_S;

	s_detect_control.elapsed += dt;
	if (s_detect_control.elapsed < MOTOR_DETECT_FLUX_ALIGN_S) {
		float ramp = Motor_Detect_Clamp(
			s_detect_control.elapsed / MOTOR_DETECT_FLUX_ALIGN_S, 0.0f, 1.0f);
		Motor_DetectControl_RunCurrent(motor, 0.0f,
			s_detect_control.current_a * ramp, 0.0f,
			MOTOR_DETECT_VOLTAGE_LIMIT_V, dt, NULL, NULL);
		return;
	}

	if (!s_detect_control.rotation_started) {
		Motor_DetectControl_Reset(motor);
		s_detect_control.rotation_started = true;
		s_detect_control.theta = 0.0f;
		s_detect_control.electrical_rpm_now = 0.0f;
	}

	rotation_time = s_detect_control.elapsed - MOTOR_DETECT_FLUX_ALIGN_S;
	transition = Motor_Detect_Clamp(rotation_time / 0.5f, 0.0f, 1.0f);
	speed_ramp = Motor_Detect_Clamp(rotation_time / MOTOR_DETECT_FLUX_RAMP_S,
		0.0f, 1.0f);
	s_detect_control.electrical_rpm_now =
		s_detect_control.electrical_rpm_target * speed_ramp;
	s_detect_control.theta += RPM2RADPS_f(s_detect_control.electrical_rpm_now) * dt;
	utils_norm_angle_rad(&s_detect_control.theta);

	id_target = s_detect_control.current_a * (1.0f - transition);
	iq_target = s_detect_control.current_a * transition;
	Motor_DetectControl_RunCurrent(motor, s_detect_control.theta,
		id_target, iq_target, MOTOR_DETECT_VOLTAGE_LIMIT_V, dt,
		&s_detect_control.vd, &s_detect_control.vq);

	if (Motor_Detect_PhaseCurrentPeak(motor) > MOTOR_DETECT_CURRENT_LIMIT_A) {
		Motor_Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_CURRENT);
		return;
	}

	if (rotation_time >
		(MOTOR_DETECT_FLUX_RAMP_S + MOTOR_DETECT_FLUX_SETTLE_S)) {
		float voltage_magnitude = NORM2_f(s_detect_control.vd, s_detect_control.vq);
		float current_magnitude = NORM2_f(motor->Id, motor->Iq);
		float linkage;

		omega_electrical = RPM2RADPS_f(s_detect_control.electrical_rpm_target);
		linkage = (voltage_magnitude -
			s_detect_result.resistance_ohm * current_magnitude) /
			omega_electrical -
			s_detect_result.inductance_h * current_magnitude;

		if (linkage > 1.0e-4f && linkage < 0.2f) {
			s_detect_control.sum_primary += linkage;
			s_detect_control.samples++;
		}
	}

	if (rotation_time >= total_rotation_time) {
		if (s_detect_control.samples < 1000U) {
			Motor_Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_FLUX);
			return;
		}

		s_detect_result.flux_linkage_wb = s_detect_control.sum_primary /
			(float)s_detect_control.samples;
		s_detect_result.flux_valid = true;
		Motor_Detect_FinishFromIsr(true, MOTOR_DETECT_ERROR_NONE);
	}
}

void Motor_Detect_DmaStep(MOTOR_PARAM_S *motor, float dt)
{
	if (!Motor_Detect_ControlIsActive() || dt <= 0.0f) {
		return;
	}
	if (s_detect_abort_requested) {
		Motor_Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_USER_ABORT);
		return;
	}

	if (motor->v_bus < 8.0f) {
		Motor_Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_FAULT);
		return;
	}

	switch (s_detect_control.stage) {
		case MOTOR_DETECT_STAGE_RESISTANCE:
			Motor_Detect_RunResistanceStep(motor, dt);
			break;
		case MOTOR_DETECT_STAGE_INDUCTANCE:
			Motor_Detect_RunInductanceStep(motor, dt);
			break;
		case MOTOR_DETECT_STAGE_FLUX:
			Motor_Detect_RunFluxStep(motor, dt);
			break;
		default:
			break;
	}
}

static void Motor_Detect_PrepareStoppedMotor(void)
{
	motor.ol_enable = false;
	motor.ol_target_rpm = 0.0f;
	motor.ol_current_rpm = 0.0f;
	motor.ol_vd_int = 0.0f;
	motor.ol_vq_int = 0.0f;
	Motor_SpeedLoop_Reset();
	Motor_Sensorless_Reset();
	Bsp_Tim_StopPwm(MOTOR1);
}

static bool Motor_Detect_RunStage(motor_detect_stage_t stage,
	float current_a, float electrical_rpm, uint32_t timeout_ms)
{
	uint32_t waited_ms = 0U;
	uint32_t primask;

	memset(&s_detect_control, 0, sizeof(s_detect_control));
	s_detect_control.stage = stage;
	s_detect_control.current_a = current_a;
	s_detect_control.electrical_rpm_target = electrical_rpm;
	s_detect_control.error = MOTOR_DETECT_ERROR_NONE;
	s_detect_control.fault = FAULT_CODE_NONE;
	Motor_DetectControl_Reset(&motor);

	primask = __get_PRIMASK();
	__disable_irq();
	s_detect_active = true;
	if (0U == primask) {
		__enable_irq();
	}
	Bsp_Tim_StartPwm(MOTOR1);

	while (!s_detect_control.done && waited_ms < timeout_ms) {
		osDelay(pdMS_TO_TICKS(1));
		waited_ms++;
	}

	if (!s_detect_control.done) {
		primask = __get_PRIMASK();
		__disable_irq();
		s_detect_active = false;
		s_detect_control.stage = MOTOR_DETECT_STAGE_IDLE;
		if (0U == primask) {
			__enable_irq();
		}
		Motor_DetectControl_OutputSafe(&motor);
		s_detect_control.error = MOTOR_DETECT_ERROR_TIMEOUT;
		s_detect_control.success = false;
	}

	Bsp_Tim_StopPwm(MOTOR1);
	osDelay(pdMS_TO_TICKS(50));
	if (!s_detect_control.success) {
		s_detect_result.error = s_detect_control.error;
		s_detect_result.fault = s_detect_control.fault;
	}
	return s_detect_control.success;
}

bool DETECT_MotorParameters(float current_a, float electrical_rpm,
	motor_detect_result_t *result)
{
	float current_kp_old = motor.ol_current_kp;
	float current_ki_old = motor.ol_current_ki;
	float voltage_limit_old = motor.ol_vq_limit;

	if (s_detect_active) {
		s_detect_result.error = MOTOR_DETECT_ERROR_BUSY;
		if (result != NULL) *result = s_detect_result;
		return false;
	}
	if (motor.ol_enable) {
		s_detect_result.error = MOTOR_DETECT_ERROR_MOTOR_RUNNING;
		if (result != NULL) *result = s_detect_result;
		return false;
	}
	if (Motor_Protect_IsFaultLatched()) {
		s_detect_result.error = MOTOR_DETECT_ERROR_FAULT;
		s_detect_result.fault = Motor_Protect_GetFault();
		if (result != NULL) *result = s_detect_result;
		return false;
	}
	if (s_detect_abort_requested) {
		memset(&s_detect_result, 0, sizeof(s_detect_result));
		s_detect_result.version = MOTOR_DETECT_RESULT_VERSION;
		s_detect_result.error = MOTOR_DETECT_ERROR_USER_ABORT;
		if (result != NULL) *result = s_detect_result;
		return false;
	}

	current_a = Motor_Detect_Clamp(fabsf(current_a), 0.3f, 2.0f);
	electrical_rpm = Motor_Detect_Clamp(fabsf(electrical_rpm), 600.0f, 3000.0f);
	memset(&s_detect_result, 0, sizeof(s_detect_result));
	s_detect_result.version = MOTOR_DETECT_RESULT_VERSION;
	s_detect_result.test_current_a = current_a;
	s_detect_result.test_electrical_rpm = electrical_rpm;

	Motor_Detect_PrepareStoppedMotor();
	PRINTF("Detect motor: calibrating current/voltage offsets\r\n");
	if (DETECT_Offset(FALSE) != 1U) {
		s_detect_result.error = s_detect_abort_requested ?
			MOTOR_DETECT_ERROR_USER_ABORT : MOTOR_DETECT_ERROR_FAULT;
		goto detect_done;
	}

	PRINTF("Detect motor: measuring phase resistance at %.2fA\r\n", current_a);
	if (!Motor_Detect_RunStage(MOTOR_DETECT_STAGE_RESISTANCE,
		current_a, electrical_rpm, 4000U)) {
		goto detect_done;
	}
	PRINTF("Detect motor: R=%.6f ohm\r\n", s_detect_result.resistance_ohm);

	PRINTF("Detect motor: measuring phase inductance with bipolar pulses\r\n");
	if (!Motor_Detect_RunStage(MOTOR_DETECT_STAGE_INDUCTANCE,
		current_a, electrical_rpm, 5000U)) {
		goto detect_done;
	}
	PRINTF("Detect motor: L=%.3f uH\r\n",
		s_detect_result.inductance_h * 1000000.0f);

	PRINTF("Detect motor: spinning at %.0f electrical RPM for flux\r\n",
		electrical_rpm);
	/* VESC 使用约 1.5ms 电流环时间常数进行磁链检测，结束后恢复用户参数。 */
	motor.ol_current_kp = s_detect_result.inductance_h / 0.0015f;
	motor.ol_current_ki = s_detect_result.resistance_ohm / 0.0015f;
	motor.ol_vq_limit = MOTOR_DETECT_VOLTAGE_LIMIT_V;
	if (!Motor_Detect_RunStage(MOTOR_DETECT_STAGE_FLUX,
		current_a, electrical_rpm, 10000U)) {
		goto detect_done;
	}
	PRINTF("Detect motor: flux=%.4f mWb\r\n",
		s_detect_result.flux_linkage_wb * 1000.0f);

	s_detect_result.error = MOTOR_DETECT_ERROR_NONE;
	s_detect_result.fault = FAULT_CODE_NONE;

detect_done:
	motor.ol_current_kp = current_kp_old;
	motor.ol_current_ki = current_ki_old;
	motor.ol_vq_limit = voltage_limit_old;
	Motor_Detect_PrepareStoppedMotor();
	if (result != NULL) {
		*result = s_detect_result;
	}
	return s_detect_result.resistance_valid &&
		s_detect_result.inductance_valid &&
		s_detect_result.flux_valid &&
		s_detect_result.error == MOTOR_DETECT_ERROR_NONE;
}

bool DETECT_CalculatePolePairs(float electrical_rpm, float mechanical_rpm,
	float *pole_pairs)
{
	float ratio;
	float rounded;

	if (fabsf(electrical_rpm) < 1.0f || fabsf(mechanical_rpm) < 1.0f) {
		s_detect_result.error = MOTOR_DETECT_ERROR_MECHANICAL_REFERENCE;
		return false;
	}

	ratio = fabsf(electrical_rpm / mechanical_rpm);
	rounded = floorf(ratio + 0.5f);
	if (rounded < 1.0f || rounded > 50.0f ||
		fabsf(ratio - rounded) > rounded * 0.15f) {
		s_detect_result.error = MOTOR_DETECT_ERROR_MECHANICAL_REFERENCE;
		return false;
	}

	s_detect_result.pole_pairs = rounded;
	s_detect_result.pole_pairs_valid = true;
	s_detect_result.error = MOTOR_DETECT_ERROR_NONE;
	if (pole_pairs != NULL) {
		*pole_pairs = rounded;
	}
	return true;
}

void DETECT_GetResult(motor_detect_result_t *result)
{
	if (result != NULL) {
		*result = s_detect_result;
	}
}

static bool Motor_Detect_ResultParametersValid(const motor_detect_result_t *result)
{
	return result->version == MOTOR_DETECT_RESULT_VERSION &&
		result->resistance_valid && result->resistance_ohm >= 0.005f &&
		result->resistance_ohm <= 10.0f &&
		result->inductance_valid && result->inductance_h >= 5.0e-6f &&
		result->inductance_h <= 0.05f &&
		result->flux_valid && result->flux_linkage_wb >= 1.0e-4f &&
		result->flux_linkage_wb <= 0.2f;
}

static void Motor_Detect_ApplyRuntime(const motor_detect_result_t *result)
{
	gpstMotorCfg->foc_motor_r = result->resistance_ohm;
	gpstMotorCfg->foc_motor_l = result->inductance_h;
	gpstMotorCfg->foc_motor_flux_linkage = result->flux_linkage_wb;
	motor.sensorless_motor_r = result->resistance_ohm;
	motor.sensorless_motor_l = result->inductance_h;
	motor.sensorless_flux_linkage = result->flux_linkage_wb;
	if (result->pole_pairs_valid) {
		motor.ol_pole_pairs = result->pole_pairs;
	}
}

bool DETECT_ApplyResult(bool save)
{
	if (!Motor_Detect_ResultParametersValid(&s_detect_result) || motor.ol_enable) {
		return false;
	}

	Motor_Detect_ApplyRuntime(&s_detect_result);
	Motor_Sensorless_Reset();
	Motor_SpeedLoop_Reset();
	if (save) {
		EfErrCode error = ef_set_env_blob(MOTOR_DETECT_FLASH_KEY,
			&s_detect_result, sizeof(s_detect_result));
		if (error != EF_NO_ERR) {
			return false;
		}
		ef_save_env();
	}
	return true;
}

void Motor_Detect_LoadStoredParameters(void)
{
	motor_detect_result_t stored;
	size_t saved_length = 0U;

	if (ef_get_env_blob(MOTOR_DETECT_FLASH_KEY, &stored, sizeof(stored),
		&saved_length) == sizeof(stored) &&
		saved_length == sizeof(stored) &&
		Motor_Detect_ResultParametersValid(&stored)) {
		s_detect_result = stored;
		Motor_Detect_ApplyRuntime(&s_detect_result);
		PRINTF("Loaded detected motor: R=%.6f L=%.3fuH flux=%.4fmWb\r\n",
			s_detect_result.resistance_ohm,
			s_detect_result.inductance_h * 1000000.0f,
			s_detect_result.flux_linkage_wb * 1000.0f);
	}
}

const char *DETECT_GetErrorName(motor_detect_error_t error)
{
	switch (error) {
		case MOTOR_DETECT_ERROR_NONE: return "NONE";
		case MOTOR_DETECT_ERROR_BUSY: return "BUSY";
		case MOTOR_DETECT_ERROR_MOTOR_RUNNING: return "MOTOR_RUNNING";
		case MOTOR_DETECT_ERROR_FAULT: return "FAULT";
		case MOTOR_DETECT_ERROR_TIMEOUT: return "TIMEOUT";
		case MOTOR_DETECT_ERROR_CURRENT: return "CURRENT_TOO_HIGH";
		case MOTOR_DETECT_ERROR_RESISTANCE: return "RESISTANCE_INVALID";
		case MOTOR_DETECT_ERROR_INDUCTANCE: return "INDUCTANCE_INVALID";
		case MOTOR_DETECT_ERROR_FLUX: return "FLUX_INVALID";
		case MOTOR_DETECT_ERROR_MECHANICAL_REFERENCE:
			return "MECHANICAL_REFERENCE_INVALID";
		case MOTOR_DETECT_ERROR_USER_ABORT: return "USER_ABORT";
		default: return "UNKNOWN";
	}
}

void DETECT_Abort(void)
{
	uint32_t primask;

	s_detect_abort_requested = true;
	if (!s_detect_active) {
		return;
	}

	/* 撤销检测互斥并立即关闭桥臂，使后台检测期间的 olstop 可以生效。 */
	primask = __get_PRIMASK();
	__disable_irq();
	if (Motor_Detect_ControlIsActive()) {
		Motor_DetectControl_OutputSafe(&motor);
		s_detect_control.success = false;
		s_detect_control.error = MOTOR_DETECT_ERROR_USER_ABORT;
		s_detect_control.done = true;
	}
	s_detect_control.stage = MOTOR_DETECT_STAGE_IDLE;
	s_detect_active = false;
	if (0U == primask) {
		__enable_irq();
	}
	Bsp_Tim_StopPwm(MOTOR1);
}

void DETECT_ClearAbortRequest(void)
{
	if (!s_detect_active) {
		s_detect_abort_requested = false;
	}
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
	DETECT_ClearAbortRequest();
	DETECT_Offset(TRUE);
	return;
}
#ifdef __cplusplus
}
#endif
