/*
 * @Description: 检测参数 — 偏置校准、霍尔检测、VESC 风格 R/L/磁链自动测量
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

/* ---- 电流与电压安全限幅 ---- */
#define MOTOR_DETECT_CURRENT_LIMIT_A        (4.0f)
#define MOTOR_DETECT_CURRENT_HARD_LIMIT_A   (8.0f)
#define MOTOR_DETECT_CURRENT_TRIP_S         (0.001f)
#define MOTOR_DETECT_MIN_BUS_VOLTAGE_V      (8.0f)
#define MOTOR_DETECT_VOLTAGE_LIMIT_FLUX_V   (8.0f)

/* ---- 电阻检测 ---- */
#define MOTOR_DETECT_RESISTANCE_VOLTAGE_V   (3.0f)
#define MOTOR_DETECT_RESISTANCE_RAMP_S      (0.8f)
#define MOTOR_DETECT_RESISTANCE_SETTLE_S    (0.2f)
#define MOTOR_DETECT_RESISTANCE_SAMPLE_S    (0.5f)
#define MOTOR_DETECT_RESISTANCE_MIN_CURRENT_RATIO (0.25f)
#define MOTOR_DETECT_RESISTANCE_MIN_SAMPLES (100U)
#define MOTOR_DETECT_RESISTANCE_MIN_OHM     (0.005f)
#define MOTOR_DETECT_RESISTANCE_MAX_OHM     (10.0f)
#define MOTOR_DETECT_RESISTANCE_FIT_POINTS  (3U)
#define MOTOR_DETECT_RESISTANCE_FIT_MIN_A   (0.8f)
#define MOTOR_DETECT_RESISTANCE_FIT_MAX_A   (1.6f)
#define MOTOR_DETECT_RESISTANCE_FIT_MIN_R2  (0.95f)

/* ---- 电感检测 ---- */
#define MOTOR_DETECT_INDUCTANCE_ALIGN_V     (3.0f)
#define MOTOR_DETECT_INDUCTANCE_ZERO_V      (2.0f)
#define MOTOR_DETECT_INDUCTANCE_ALIGN_S     (0.8f)
#define MOTOR_DETECT_INDUCTANCE_ZERO_S      (0.2f)
#define MOTOR_DETECT_INDUCTANCE_PULSE_V     (0.5f)
#define MOTOR_DETECT_INDUCTANCE_HALF_CYCLES (8U)
#define MOTOR_DETECT_INDUCTANCE_TARGET_PULSES (120U)
#define MOTOR_DETECT_INDUCTANCE_MIN_SLOPE   (1.0f)
#define MOTOR_DETECT_INDUCTANCE_MIN_H       (5.0e-6f)
#define MOTOR_DETECT_INDUCTANCE_MAX_H       (0.05f)
#define MOTOR_DETECT_INDUCTANCE_DQ_FACTOR   (2.0f)  /* 脉冲半量 -> FOC dq 相电感 */
#define MOTOR_DETECT_INDUCTANCE_STABILITY_SCALE (0.9f) /* 略低估，观测器更稳定 */

/* ---- 磁链检测 ---- */
#define MOTOR_DETECT_FLUX_ALIGN_S           (1.0f)
#define MOTOR_DETECT_FLUX_RAMP_S            (8.0f)
#define MOTOR_DETECT_FLUX_ID_IQ_TRANSITION_S (1.0f)
#define MOTOR_DETECT_FLUX_MIN_CURRENT_A     (1.5f)
#define MOTOR_DETECT_FLUX_MIN_RUN_DUTY      (0.03f)
#define MOTOR_DETECT_FLUX_DUTY_RISE_FACTOR  (1.5f)
#define MOTOR_DETECT_FLUX_DUTY_COLLAPSE_RATIO (0.70f)
#define MOTOR_DETECT_FLUX_MIN_SAMPLES       (1000U)
#define MOTOR_DETECT_FLUX_MIN_WB            (1.0e-4f)
#define MOTOR_DETECT_FLUX_MAX_WB            (0.2f)
#define MOTOR_DETECT_FLUX_COAST_SETTLE_S    (0.01f)
#define MOTOR_DETECT_FLUX_COAST_SAMPLE_S    (0.50f)
#define MOTOR_DETECT_FLUX_COAST_MIN_ERPM    (100.0f)
#define MOTOR_DETECT_FLUX_OMEGA_FILTER      (0.02f)
#define MOTOR_DETECT_FLUX_MIN_BEMF_V        (0.20f)
#define MOTOR_DETECT_FLUX_VF_BOOST_V        (1.0f)
#define MOTOR_DETECT_FLUX_VF_V_PER_HZ       (0.057f)

/* ---- 通用 ---- */
#define MOTOR_DETECT_PWM_COOLDOWN_MS        (50U)  /* PWM 关断后等待功率级彻底进入高阻 */
#define MOTOR_DETECT_VESC_CURRENT_TC_S      (0.0015f)  /* VESC 1.5ms 电流环时间常数 */
#define MOTOR_DETECT_MIN_CURRENT_A          (0.3f)
#define MOTOR_DETECT_MAX_CURRENT_A          (2.0f)
#define MOTOR_DETECT_MIN_ELECTRICAL_RPM     (600.0f)
#define MOTOR_DETECT_MAX_ELECTRICAL_RPM     (3000.0f)

/* ---- 极对数计算 ---- */
#define MOTOR_DETECT_POLE_PAIRS_MIN         (1.0f)
#define MOTOR_DETECT_POLE_PAIRS_MAX         (50.0f)
#define MOTOR_DETECT_POLE_PAIRS_RATIO_TOL   (0.15f)

/* ---- 各阶段超时 ---- */
#define MOTOR_DETECT_TIMEOUT_RESISTANCE_MS  (4000U)
#define MOTOR_DETECT_TIMEOUT_INDUCTANCE_MS  (5000U)
#define MOTOR_DETECT_TIMEOUT_FLUX_MS        (18000U)

/* ---- 偏置校准 ---- */
#define MOTOR_DETECT_OFFSET_SAMPLES         (1000U)
#define MOTOR_DETECT_OFFSET_MAX_PERCENT     (5.0f)  /* 偏移超过此比例提示硬件异常 */

typedef enum {
	MOTOR_DETECT_STAGE_IDLE = 0,
	MOTOR_DETECT_STAGE_OFFSET,
	MOTOR_DETECT_STAGE_RESISTANCE,
	MOTOR_DETECT_STAGE_INDUCTANCE,
	MOTOR_DETECT_STAGE_FLUX,
} motor_detect_stage_t;

/*
 * 检测控制结构。
 *
 * ISR 会读写的成员加 volatile；仅任务态访问的成员不加。
 * accum_a / accum_b 在不同阶段含义不同：
 *   电阻阶段：accum_a = 累计 V, accum_b = 累计 I
 *   电感阶段：accum_a = 累计电感值(H), accum_b = 未使用
 *   磁链阶段：accum_a = 累计磁链(Wb), accum_b = 未使用
 */
typedef struct {
	/* ---- ISR 可读写 (volatile) ---- */
	volatile motor_detect_stage_t stage;
	volatile bool done;
	volatile bool success;
	volatile motor_detect_error_t error;
	volatile mc_fault_code fault;
	/* ---- 仅 ISR 或仅任务态，不同步访问 ---- */
	float elapsed;
	float current_a;
	float electrical_rpm_target;
	float electrical_rpm_now;
	float theta;
	float vd;
	float vq;
	float accum_a;
	float accum_b;
	float resistance_voltage_avg;
	float resistance_current_avg;
	float max_phase_current;
	float overcurrent_elapsed;
	float flux_duty_still;
	float flux_duty_now;
	float flux_duty_max;
	float flux_coast_elapsed;
	float flux_bemf_angle_last;
	float flux_omega_e;
	uint32_t samples;
	uint32_t pulse_position;
	uint32_t pulse_count;
	uint32_t valid_pulses;
	float pulse_start_current;
	bool pulse_started;
	bool rotation_started;
	bool flux_coast_started;
	bool flux_bemf_angle_valid;
} motor_detect_control_t;

static motor_detect_control_t s_detect_control;
static motor_detect_result_t s_detect_result = {
	.version = MOTOR_DETECT_RESULT_VERSION,
};

/*
 * Fit V = R * I + Voffset. The slope rejects the approximately constant
 * inverter voltage drop that makes a single-point V/I result too high.
 */
static bool Motor_Detect_FitResistance(const float *current,
	const float *voltage, uint32_t count, float *resistance,
	float *voltage_offset, float *r_squared)
{
	float sum_i = 0.0f;
	float sum_v = 0.0f;
	float sum_ii = 0.0f;
	float sum_iv = 0.0f;
	float ss_tot = 0.0f;
	float ss_res = 0.0f;
	float denominator;
	float mean_v;

	if (count < 2U) return false;
	for (uint32_t i = 0U; i < count; i++) {
		sum_i += current[i];
		sum_v += voltage[i];
		sum_ii += current[i] * current[i];
		sum_iv += current[i] * voltage[i];
	}
	denominator = (float)count * sum_ii - sum_i * sum_i;
	if (fabsf(denominator) < 1.0e-6f) return false;

	*resistance = ((float)count * sum_iv - sum_i * sum_v) / denominator;
	*voltage_offset = (sum_v - *resistance * sum_i) / (float)count;
	mean_v = sum_v / (float)count;
	for (uint32_t i = 0U; i < count; i++) {
		float error = voltage[i] - (*resistance * current[i] + *voltage_offset);
		float deviation = voltage[i] - mean_v;
		ss_res += error * error;
		ss_tot += deviation * deviation;
	}
	*r_squared = ss_tot > 1.0e-9f ? 1.0f - ss_res / ss_tot : 0.0f;
	return true;
}
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* ---- 偏置校准内部采样辅助 ---- */

/*
 * 对单个相采样 @p samples 次，累加电流和电压。
 * 返回 false 表示被用户中止，调用方应跳转到清理点。
 */
static bool Detect_SamplePhase(uint32_t phase, uint32_t samples,
                               float *current_acc, float *voltage_acc)
{
	for (uint32_t i = 0U; i < samples; i++) {
		if (s_detect_abort_requested) {
			return false;
		}
		switch (phase) {
		case 0:
			*current_acc += GET_CURRENT1();
			*voltage_acc += ADC_VOLTS(ADC_IND_SENS1);
			break;
		case 1:
			*current_acc += GET_CURRENT2();
			*voltage_acc += ADC_VOLTS(ADC_IND_SENS2);
			break;
		default:
			*current_acc += GET_CURRENT3();
			*voltage_acc += ADC_VOLTS(ADC_IND_SENS3);
			break;
		}
		osDelay(pdMS_TO_TICKS(1));
	}
	return true;
}

/**
 * Calibrate voltage and current offsets. For the observer to work at low modulation it
 * is very important to get all current and voltage offsets right. Therefore we store
 * the offsets for when the motor is undriven and when it is driven separately. The
 * motor is driven at 50% modulation on all phases when measuring the driven offset, which
 * corresponds to space-vector modulation with 0 amplitude.
 *
 * cal_undriven: true 时同时校准未驱动相电压偏移（需要电机静止）。
 *
 * return:
 *   0: 被用户中止
 *   1: 成功
 */
ULONG DETECT_Offset(BOOL cal_undriven)
{
	const uint32_t samples = MOTOR_DETECT_OFFSET_SAMPLES;
	float current_sum[3] = {0.0f, 0.0f, 0.0f};
	float voltage_sum[3] = {0.0f, 0.0f, 0.0f};
	bool aborted = false;
	float v_avg;

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

	/* ---- Phase A ---- */
	Bsp_Tim_StopPwm(MOTOR1);
	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
	osDelay(pdMS_TO_TICKS(1));
	PRINTF("Detect offset: phase A\r\n");
	if (!Detect_SamplePhase(0U, samples, &current_sum[0], &voltage_sum[0])) {
		aborted = true;
		goto detect_offset_finish;
	}

	/* ---- Phase B ---- */
	Bsp_Tim_StopPwm(MOTOR1);
	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
	osDelay(pdMS_TO_TICKS(1));
	PRINTF("Detect offset: phase B\r\n");
	if (!Detect_SamplePhase(1U, samples, &current_sum[1], &voltage_sum[1])) {
		aborted = true;
		goto detect_offset_finish;
	}

	/* ---- Phase C ---- */
	Bsp_Tim_StopPwm(MOTOR1);
	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
	osDelay(pdMS_TO_TICKS(1));
	PRINTF("Detect offset: phase C\r\n");
	if (!Detect_SamplePhase(2U, samples, &current_sum[2], &voltage_sum[2])) {
		aborted = true;
		goto detect_offset_finish;
	}

	Bsp_Tim_StopPwm(MOTOR1);

	motor.currentOffset[0] = current_sum[0] / (float)samples;
	motor.currentOffset[1] = current_sum[1] / (float)samples;
	motor.currentOffset[2] = current_sum[2] / (float)samples;

	voltage_sum[0] /= (float)samples;
	voltage_sum[1] /= (float)samples;
	voltage_sum[2] /= (float)samples;
	v_avg = (voltage_sum[0] + voltage_sum[1] + voltage_sum[2]) / 3.0f;

	motor.voltOffset[0] = voltage_sum[0] - v_avg;
	motor.voltOffset[1] = voltage_sum[1] - v_avg;
	motor.voltOffset[2] = voltage_sum[2] - v_avg;

	PRINTF("ADC_RAW [%d] [%d] [%d]\r\n",
	       (int)GET_CURRENT1(), (int)GET_CURRENT2(), (int)GET_CURRENT3());
	PRINTF("Current Offset [%f] [%f] [%f]\r\n",
	       motor.currentOffset[0], motor.currentOffset[1], motor.currentOffset[2]);
	PRINTF("Volt Offset [%f] [%f] [%f]\r\n",
	       motor.voltOffset[0], motor.voltOffset[1], motor.voltOffset[2]);

	/* 偏移量过大通常意味着硬件问题（运放偏置、参考电压异常等）。 */
	{
		float adc_range = 4096.0f;
		float abs_max;

		/* Zero current is centered at half the ADC range, not at zero. */
		abs_max = fabsf(motor.currentOffset[0] - adc_range * 0.5f);
		if (fabsf(motor.currentOffset[1] - adc_range * 0.5f) > abs_max)
			abs_max = fabsf(motor.currentOffset[1] - adc_range * 0.5f);
		if (fabsf(motor.currentOffset[2] - adc_range * 0.5f) > abs_max)
			abs_max = fabsf(motor.currentOffset[2] - adc_range * 0.5f);
		if (abs_max > adc_range * MOTOR_DETECT_OFFSET_MAX_PERCENT / 100.0f) {
			PRINTF("WARNING: current offset > %.0f%% ADC range — check hardware\r\n",
			       (double)MOTOR_DETECT_OFFSET_MAX_PERCENT);
		}
	}

	/* Measure undriven offsets */
	if (cal_undriven) {
		PRINTF("Detect offset: undriven voltage\r\n");
		osDelay(pdMS_TO_TICKS(10));

		voltage_sum[0] = 0.0f; voltage_sum[1] = 0.0f; voltage_sum[2] = 0.0f;

		for (uint32_t i = 0U; i < samples; i++) {
			if (s_detect_abort_requested) {
				aborted = true;
				break;
			}
			v_avg = (ADC_VOLTS(ADC_IND_SENS1) +
			         ADC_VOLTS(ADC_IND_SENS2) +
			         ADC_VOLTS(ADC_IND_SENS3)) / 3.0f;
			voltage_sum[0] += ADC_VOLTS(ADC_IND_SENS1) - v_avg;
			voltage_sum[1] += ADC_VOLTS(ADC_IND_SENS2) - v_avg;
			voltage_sum[2] += ADC_VOLTS(ADC_IND_SENS3) - v_avg;
			osDelay(pdMS_TO_TICKS(1));
		}
		if (aborted) goto detect_offset_finish;
		Bsp_Tim_StopPwm(MOTOR1);

		voltage_sum[0] /= (float)samples;
		voltage_sum[1] /= (float)samples;
		voltage_sum[2] /= (float)samples;

		motor.voltundrivenOffset[0] = voltage_sum[0];
		motor.voltundrivenOffset[1] = voltage_sum[1];
		motor.voltundrivenOffset[2] = voltage_sum[2];
	}

	// TODO 看门狗时间恢复

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

/* Reject isolated PWM/ADC spikes, but retain an immediate hard cutoff. */
static bool Motor_Detect_CurrentTooHigh(const MOTOR_PARAM_S *motor, float dt)
{
	float peak = Motor_Detect_PhaseCurrentPeak(motor);

	if (peak > s_detect_control.max_phase_current) {
		s_detect_control.max_phase_current = peak;
	}
	if (peak >= MOTOR_DETECT_CURRENT_HARD_LIMIT_A) {
		return true;
	}
	if (peak > MOTOR_DETECT_CURRENT_LIMIT_A) {
		s_detect_control.overcurrent_elapsed += dt;
	} else {
		s_detect_control.overcurrent_elapsed = 0.0f;
	}
	return s_detect_control.overcurrent_elapsed >= MOTOR_DETECT_CURRENT_TRIP_S;
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

/* ========================================================================
 * 电阻检测：固定电角度 θ=0，Id 从 0 斜坡上升到目标电流。
 * 在最后的稳态采样窗口内累计 V/I 矢量模长，R = avg(|V|)/avg(|I|)。
 * ======================================================================== */
static void Motor_Detect_RunResistanceStep(MOTOR_PARAM_S *motor, float dt)
{
	float total_time = MOTOR_DETECT_RESISTANCE_RAMP_S +
		MOTOR_DETECT_RESISTANCE_SETTLE_S +
		MOTOR_DETECT_RESISTANCE_SAMPLE_S;

	s_detect_control.elapsed += dt;
	float ramp = Motor_Detect_Clamp(
		s_detect_control.elapsed / MOTOR_DETECT_RESISTANCE_RAMP_S, 0.0f, 1.0f);
	float id_target = s_detect_control.current_a * ramp;

	Motor_DetectControl_RunCurrent(motor, 0.0f, id_target, 0.0f,
		MOTOR_DETECT_RESISTANCE_VOLTAGE_V, dt,
		&s_detect_control.vd, &s_detect_control.vq);

	if (Motor_Detect_CurrentTooHigh(motor, dt)) {
		Motor_Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_CURRENT);
		return;
	}

	if (s_detect_control.elapsed >
		(MOTOR_DETECT_RESISTANCE_RAMP_S + MOTOR_DETECT_RESISTANCE_SETTLE_S)) {
		float current_mag = NORM2_f(motor->Id, motor->Iq);
		float voltage_mag = NORM2_f(s_detect_control.vd, s_detect_control.vq);
		if (current_mag > s_detect_control.current_a *
			MOTOR_DETECT_RESISTANCE_MIN_CURRENT_RATIO) {
			s_detect_control.accum_a += voltage_mag;
			s_detect_control.accum_b += current_mag;
			s_detect_control.samples++;
		}
	}

	if (s_detect_control.elapsed >= total_time) {
		if (s_detect_control.samples < MOTOR_DETECT_RESISTANCE_MIN_SAMPLES ||
			s_detect_control.accum_b <= 0.0f) {
			Motor_Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_RESISTANCE);
			return;
		}

		float resistance = s_detect_control.accum_a /
			s_detect_control.accum_b;
		s_detect_control.resistance_voltage_avg =
			s_detect_control.accum_a / (float)s_detect_control.samples;
		s_detect_control.resistance_current_avg =
			s_detect_control.accum_b / (float)s_detect_control.samples;
		if (resistance < MOTOR_DETECT_RESISTANCE_MIN_OHM ||
			resistance > MOTOR_DETECT_RESISTANCE_MAX_OHM) {
			Motor_Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_RESISTANCE);
			return;
		}

		s_detect_result.resistance_ohm = resistance;
		s_detect_result.resistance_valid = true;
		Motor_Detect_FinishFromIsr(true, MOTOR_DETECT_ERROR_NONE);
	}
}

/* ========================================================================
 * 电感检测：固定转子后在 D 轴施加 ±0.5V 双极性短脉冲。
 * 每半脉冲量测 di/dt，按 V = R*I + L*dI/dt 反算 L。
 * 正负交替避免单向转矩累积。
 * ======================================================================== */
static void Motor_Detect_RunInductanceStep(MOTOR_PARAM_S *motor, float dt)
{
	float pulse_time_start = MOTOR_DETECT_INDUCTANCE_ALIGN_S +
		MOTOR_DETECT_INDUCTANCE_ZERO_S;

	s_detect_control.elapsed += dt;

	/* 阶段 1：预对齐 — 吸住转子 */
	if (s_detect_control.elapsed < MOTOR_DETECT_INDUCTANCE_ALIGN_S) {
		float ramp = Motor_Detect_Clamp(
			s_detect_control.elapsed / MOTOR_DETECT_INDUCTANCE_ALIGN_S,
			0.0f, 1.0f);
		Motor_DetectControl_RunCurrent(motor, 0.0f,
			s_detect_control.current_a * ramp, 0.0f,
			MOTOR_DETECT_INDUCTANCE_ALIGN_V, dt, NULL, NULL);
		return;
	}

	/* 阶段 2：短暂零电流，让电感电流清零 */
	if (s_detect_control.elapsed < pulse_time_start) {
		Motor_DetectControl_RunCurrent(motor, 0.0f, 0.0f, 0.0f,
			MOTOR_DETECT_INDUCTANCE_ZERO_V, dt, NULL, NULL);
		return;
	}

	/* 阶段 3：双极性脉冲 */
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
			/* 使用已配置的电机电阻而非刚测的值：死区会导致小电流下 R 偏高 3-4 倍。 */
			float r_drop = motor->sensorless_motor_r * current_mid;
			float effective_voltage = voltage - r_drop;

			if (fabsf(slope) > MOTOR_DETECT_INDUCTANCE_MIN_SLOPE) {
				float inductance = effective_voltage / slope;
				if (inductance > MOTOR_DETECT_INDUCTANCE_MIN_H &&
					inductance < MOTOR_DETECT_INDUCTANCE_MAX_H) {
					s_detect_control.accum_a += inductance;
					s_detect_control.valid_pulses++;
				}
			}

			s_detect_control.pulse_count++;
			s_detect_control.pulse_position = 0U;
		} else {
			s_detect_control.pulse_position++;
		}
	}

	if (Motor_Detect_CurrentTooHigh(motor, dt)) {
		Motor_Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_CURRENT);
		return;
	}

	if (s_detect_control.valid_pulses >= MOTOR_DETECT_INDUCTANCE_TARGET_PULSES ||
		s_detect_control.pulse_count >= MOTOR_DETECT_INDUCTANCE_TARGET_PULSES * 3U) {
		if (s_detect_control.valid_pulses <
			MOTOR_DETECT_INDUCTANCE_TARGET_PULSES / 2U) {
			Motor_Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_INDUCTANCE);
			return;
		}

		float inductance = MOTOR_DETECT_INDUCTANCE_DQ_FACTOR *
			MOTOR_DETECT_INDUCTANCE_STABILITY_SCALE *
			s_detect_control.accum_a / (float)s_detect_control.valid_pulses;
		s_detect_result.inductance_h = inductance;
		s_detect_result.inductance_valid = true;
		Motor_Detect_FinishFromIsr(true, MOTOR_DETECT_ERROR_NONE);
	}
}

/* ========================================================================
 * 磁链检测：先开环加速，随后关闭六管，在自由滑行窗口中由三相
 * 反电势矢量的幅值和真实角速度计算 λ = |Vαβ| / |ωe|。
 * ======================================================================== */
static void Motor_Detect_RunFluxStep(MOTOR_PARAM_S *motor, float dt)
{
	s_detect_control.elapsed += dt;

	/* 阶段 1：固定 Id 预对齐 */
	if (s_detect_control.elapsed < MOTOR_DETECT_FLUX_ALIGN_S) {
		float ramp = Motor_Detect_Clamp(
			s_detect_control.elapsed / MOTOR_DETECT_FLUX_ALIGN_S,
			0.0f, 1.0f);
		Motor_DetectControl_RunCurrent(motor, 0.0f,
			s_detect_control.current_a * ramp, 0.0f,
			MOTOR_DETECT_VOLTAGE_LIMIT_FLUX_V, dt,
			&s_detect_control.vd, &s_detect_control.vq);
		if (motor->v_bus > 1.0f && ramp > 0.9f) {
			s_detect_control.flux_duty_still =
				NORM2_f(s_detect_control.vd, s_detect_control.vq) / motor->v_bus;
		}
		return;
	}

	/* 阶段 2-4：Id→Iq 过渡 → 转速斜坡 → 稳态采样 */
	if (!s_detect_control.rotation_started) {
		Motor_DetectControl_Reset(motor);
		s_detect_control.rotation_started = true;
		s_detect_control.theta = 0.0f;
		s_detect_control.electrical_rpm_now = 0.0f;
	}

	float rotation_time = s_detect_control.elapsed - MOTOR_DETECT_FLUX_ALIGN_S;

	/* Stage 3: all MOSFETs are off. Derive both voltage and true electrical
	 * speed from the freely-decaying back-EMF vector. */
	if (s_detect_control.flux_coast_started) {
		float v_alpha;
		float v_beta;
		float voltage_mag;
		float bemf_angle;
		float delta_angle;
		float omega_instant;
		float linkage;

		s_detect_control.flux_coast_elapsed += dt;
		clarke_transform(motor->va, motor->vb, motor->vc,
			&v_alpha, &v_beta);
		voltage_mag = NORM2_f(v_alpha, v_beta);
		bemf_angle = utils_fast_atan2(v_beta, v_alpha);

		if (!s_detect_control.flux_bemf_angle_valid) {
			s_detect_control.flux_bemf_angle_last = bemf_angle;
			s_detect_control.flux_bemf_angle_valid = true;
			return;
		}

		delta_angle = bemf_angle - s_detect_control.flux_bemf_angle_last;
		while (delta_angle > M_PI) delta_angle -= 2.0f * M_PI;
		while (delta_angle < -M_PI) delta_angle += 2.0f * M_PI;
		s_detect_control.flux_bemf_angle_last = bemf_angle;
		omega_instant = delta_angle / dt;

		if (fabsf(s_detect_control.flux_omega_e) < 1.0e-6f) {
			s_detect_control.flux_omega_e = omega_instant;
		} else {
			s_detect_control.flux_omega_e += MOTOR_DETECT_FLUX_OMEGA_FILTER *
				(omega_instant - s_detect_control.flux_omega_e);
		}
		s_detect_control.electrical_rpm_now =
			fabsf(RADPS2RPM_f(s_detect_control.flux_omega_e));

		if (s_detect_control.flux_coast_elapsed >
			MOTOR_DETECT_FLUX_COAST_SETTLE_S &&
			s_detect_control.electrical_rpm_now >
			MOTOR_DETECT_FLUX_COAST_MIN_ERPM &&
			voltage_mag > MOTOR_DETECT_FLUX_MIN_BEMF_V) {
			linkage = voltage_mag / fabsf(s_detect_control.flux_omega_e);
			if (linkage > MOTOR_DETECT_FLUX_MIN_WB &&
				linkage < MOTOR_DETECT_FLUX_MAX_WB) {
				s_detect_control.accum_a += linkage;
				s_detect_control.accum_b += voltage_mag;
				s_detect_control.samples++;
			}
		}

		if (s_detect_control.flux_coast_elapsed >=
			MOTOR_DETECT_FLUX_COAST_SAMPLE_S) {
			if (s_detect_control.samples < MOTOR_DETECT_FLUX_MIN_SAMPLES) {
				Motor_Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_FLUX);
				return;
			}
			s_detect_result.flux_linkage_wb =
				s_detect_control.accum_a / (float)s_detect_control.samples;
			s_detect_control.resistance_voltage_avg =
				s_detect_control.accum_b / (float)s_detect_control.samples;
			s_detect_result.flux_valid = true;
			Motor_Detect_FinishFromIsr(true, MOTOR_DETECT_ERROR_NONE);
		}
		return;
	}

	/* Stage 2: accelerate the rotating current vector. */
	float transition = Motor_Detect_Clamp(
		rotation_time / MOTOR_DETECT_FLUX_ID_IQ_TRANSITION_S, 0.0f, 1.0f);
	float speed_ramp = Motor_Detect_Clamp(
		rotation_time / MOTOR_DETECT_FLUX_RAMP_S, 0.0f, 1.0f);

	s_detect_control.electrical_rpm_now =
		s_detect_control.electrical_rpm_target * speed_ramp;
	s_detect_control.theta +=
		RPM2RADPS_f(s_detect_control.electrical_rpm_now) * dt;
	utils_norm_angle_rad(&s_detect_control.theta);

	/* V/F acceleration is more tolerant of temporary angle error than the
	 * q-axis current loop and is sufficient for unloaded coast-down testing. */
	float electrical_hz = s_detect_control.electrical_rpm_now / 60.0f;
	float vq_target = (MOTOR_DETECT_FLUX_VF_BOOST_V +
		MOTOR_DETECT_FLUX_VF_V_PER_HZ * electrical_hz) * transition;
	s_detect_control.vd = 0.0f;
	s_detect_control.vq = vq_target;
	Motor_DetectControl_RunVoltage(motor, s_detect_control.theta,
		0.0f, vq_target, MOTOR_DETECT_VOLTAGE_LIMIT_FLUX_V);
	if (motor->v_bus > 1.0f) {
		s_detect_control.flux_duty_now =
			NORM2_f(s_detect_control.vd, s_detect_control.vq) / motor->v_bus;
		if (s_detect_control.flux_duty_now > s_detect_control.flux_duty_max) {
			s_detect_control.flux_duty_max = s_detect_control.flux_duty_now;
		}
	}

	if (Motor_Detect_CurrentTooHigh(motor, dt)) {
		Motor_Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_CURRENT);
		return;
	}

	/* Require clear evidence that the rotor produced more than locked-rotor
	 * winding drop before disconnecting the bridge for coast sampling. */
	if (rotation_time >= MOTOR_DETECT_FLUX_RAMP_S) {
		float minimum_run_duty = fmaxf(MOTOR_DETECT_FLUX_MIN_RUN_DUTY,
			s_detect_control.flux_duty_still *
			MOTOR_DETECT_FLUX_DUTY_RISE_FACTOR);
		if (s_detect_control.flux_duty_now < minimum_run_duty ||
			(s_detect_control.flux_duty_max > minimum_run_duty &&
			 s_detect_control.flux_duty_now < s_detect_control.flux_duty_max *
			 MOTOR_DETECT_FLUX_DUTY_COLLAPSE_RATIO)) {
			Motor_Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_FLUX);
			return;
		}

		Bsp_Tim_StopPwm(MOTOR1);
		PHASE_FILTER_ON();
		s_detect_control.flux_coast_started = true;
		s_detect_control.flux_coast_elapsed = 0.0f;
		s_detect_control.flux_bemf_angle_valid = false;
		s_detect_control.flux_omega_e = 0.0f;
		s_detect_control.accum_a = 0.0f;
		s_detect_control.accum_b = 0.0f;
		s_detect_control.samples = 0U;
	}
}

/* ========================================================================
 * Motor_DmaInt() 中每个检测控制周期（15kHz）调用一次。
 * ======================================================================== */
void Motor_Detect_DmaStep(MOTOR_PARAM_S *motor, float dt)
{
	if (!Motor_Detect_ControlIsActive() || dt <= 0.0f) {
		return;
	}
	if (s_detect_abort_requested) {
		Motor_Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_USER_ABORT);
		return;
	}

	if (motor->v_bus < MOTOR_DETECT_MIN_BUS_VOLTAGE_V) {
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

/* ---- 任务态：阶段调度与超时 ---- */

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

/*
 * 启动一个检测阶段，在任务态阻塞轮询直到 ISR 标记 done 或超时。
 * s_detect_control 的 ISR 字段（stage/done/success/error/fault）为 volatile，
 * 其余任务态字段用逐成员赋值代替 memset 以避免 volatile 对象的未定义行为。
 */
static bool Motor_Detect_RunStage(motor_detect_stage_t stage,
	float current_a, float electrical_rpm, uint32_t timeout_ms)
{
	uint32_t waited_ms = 0U;
	uint32_t primask;

	/* 逐成员清零任务态字段 */
	s_detect_control.stage                  = stage;
	s_detect_control.done                   = false;
	s_detect_control.success                = false;
	s_detect_control.error                  = MOTOR_DETECT_ERROR_NONE;
	s_detect_control.fault                  = FAULT_CODE_NONE;
	s_detect_control.elapsed                = 0.0f;
	s_detect_control.current_a              = current_a;
	s_detect_control.electrical_rpm_target  = electrical_rpm;
	s_detect_control.electrical_rpm_now     = 0.0f;
	s_detect_control.theta                  = 0.0f;
	s_detect_control.vd                     = 0.0f;
	s_detect_control.vq                     = 0.0f;
	s_detect_control.accum_a                 = 0.0f;
	s_detect_control.accum_b                 = 0.0f;
	s_detect_control.resistance_voltage_avg  = 0.0f;
	s_detect_control.resistance_current_avg  = 0.0f;
	s_detect_control.max_phase_current       = 0.0f;
	s_detect_control.overcurrent_elapsed     = 0.0f;
	s_detect_control.flux_duty_still         = 0.0f;
	s_detect_control.flux_duty_now           = 0.0f;
	s_detect_control.flux_duty_max           = 0.0f;
	s_detect_control.flux_coast_elapsed      = 0.0f;
	s_detect_control.flux_bemf_angle_last    = 0.0f;
	s_detect_control.flux_omega_e            = 0.0f;
	s_detect_control.samples                = 0U;
	s_detect_control.pulse_position         = 0U;
	s_detect_control.pulse_count            = 0U;
	s_detect_control.valid_pulses           = 0U;
	s_detect_control.pulse_start_current    = 0.0f;
	s_detect_control.pulse_started          = false;
	s_detect_control.rotation_started       = false;
	s_detect_control.flux_coast_started      = false;
	s_detect_control.flux_bemf_angle_valid   = false;

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
	/* 等待功率级彻底进入高阻态再返回，避免下一阶段立即重开 PWM 时直通。 */
	osDelay(pdMS_TO_TICKS(MOTOR_DETECT_PWM_COOLDOWN_MS));
	if (!s_detect_control.success) {
		s_detect_result.error = s_detect_control.error;
		s_detect_result.fault = s_detect_control.fault;
	}
	return s_detect_control.success;
}

/* ---- 公开 API ---- */

bool DETECT_MotorParameters(float current_a, float electrical_rpm,
	motor_detect_result_t *result)
{
	float current_kp_old = motor.ol_current_kp;
	float current_ki_old = motor.ol_current_ki;
	float voltage_limit_old = motor.ol_vq_limit;
	float fit_current[MOTOR_DETECT_RESISTANCE_FIT_POINTS];
	float fit_voltage[MOTOR_DETECT_RESISTANCE_FIT_POINTS];
	float resistance_fit = 0.0f;
	float inverter_voltage_offset = 0.0f;
	float resistance_fit_r2 = 0.0f;
	float flux_current_a;

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

	current_a = Motor_Detect_Clamp(fabsf(current_a),
		MOTOR_DETECT_MIN_CURRENT_A, MOTOR_DETECT_MAX_CURRENT_A);
	flux_current_a = current_a;
	if (flux_current_a < MOTOR_DETECT_FLUX_MIN_CURRENT_A) {
		flux_current_a = MOTOR_DETECT_FLUX_MIN_CURRENT_A;
	}
	electrical_rpm = Motor_Detect_Clamp(fabsf(electrical_rpm),
		MOTOR_DETECT_MIN_ELECTRICAL_RPM, MOTOR_DETECT_MAX_ELECTRICAL_RPM);
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

	PRINTF("Detect motor: fitting phase resistance at %.1f..%.1fA\r\n",
		MOTOR_DETECT_RESISTANCE_FIT_MIN_A,
		MOTOR_DETECT_RESISTANCE_FIT_MAX_A);
	for (uint32_t point = 0U;
		point < MOTOR_DETECT_RESISTANCE_FIT_POINTS; point++) {
		float ratio = (float)point /
			(float)(MOTOR_DETECT_RESISTANCE_FIT_POINTS - 1U);
		float fit_target = MOTOR_DETECT_RESISTANCE_FIT_MIN_A + ratio *
			(MOTOR_DETECT_RESISTANCE_FIT_MAX_A -
			 MOTOR_DETECT_RESISTANCE_FIT_MIN_A);

		if (!Motor_Detect_RunStage(MOTOR_DETECT_STAGE_RESISTANCE,
			fit_target, electrical_rpm, MOTOR_DETECT_TIMEOUT_RESISTANCE_MS)) {
			/* A trial stage may have published its temporary V/I result. */
			s_detect_result.resistance_valid = false;
			PRINTF("  R fit aborted: phase peak=%.3fA, over-limit=%.3fms\r\n",
				s_detect_control.max_phase_current,
				s_detect_control.overcurrent_elapsed * 1000.0f);
			goto detect_done;
		}
		fit_current[point] = s_detect_control.resistance_current_avg;
		fit_voltage[point] = s_detect_control.resistance_voltage_avg;
		PRINTF("  R fit %lu/%lu: I=%.4fA V=%.4fV V/I=%.4f ohm peak=%.3fA\r\n",
			(unsigned long)(point + 1U),
			(unsigned long)MOTOR_DETECT_RESISTANCE_FIT_POINTS,
			fit_current[point], fit_voltage[point],
			fit_voltage[point] / fit_current[point],
			s_detect_control.max_phase_current);
	}
	if (!Motor_Detect_FitResistance(fit_current, fit_voltage,
		MOTOR_DETECT_RESISTANCE_FIT_POINTS, &resistance_fit,
		&inverter_voltage_offset, &resistance_fit_r2) ||
		resistance_fit < MOTOR_DETECT_RESISTANCE_MIN_OHM ||
		resistance_fit > MOTOR_DETECT_RESISTANCE_MAX_OHM ||
		resistance_fit_r2 < MOTOR_DETECT_RESISTANCE_FIT_MIN_R2) {
		s_detect_result.resistance_valid = false;
		s_detect_result.error = MOTOR_DETECT_ERROR_RESISTANCE;
		PRINTF("Detect motor: resistance fit invalid R=%.6f R2=%.4f\r\n",
			resistance_fit, resistance_fit_r2);
		goto detect_done;
	}
	s_detect_result.resistance_ohm = resistance_fit;
	s_detect_result.resistance_valid = true;
	PRINTF("Detect motor: R=%.6f ohm, inverter offset=%.4fV, R2=%.4f\r\n",
		resistance_fit, inverter_voltage_offset, resistance_fit_r2);

	PRINTF("Detect motor: measuring phase inductance with bipolar pulses\r\n");
	if (!Motor_Detect_RunStage(MOTOR_DETECT_STAGE_INDUCTANCE,
		current_a, electrical_rpm, MOTOR_DETECT_TIMEOUT_INDUCTANCE_MS)) {
		goto detect_done;
	}
	PRINTF("Detect motor: L=%.3f uH\r\n",
		s_detect_result.inductance_h * 1000000.0f);

	PRINTF("Detect motor: accelerating to %.0f electrical RPM for coast BEMF (%.2fA, %.1fs ramp)\r\n",
		electrical_rpm, flux_current_a,
		MOTOR_DETECT_FLUX_RAMP_S);
	/* VESC 使用约 1.5ms 电流环时间常数进行磁链检测，结束后恢复用户参数。 */
	motor.ol_current_kp = s_detect_result.inductance_h /
		MOTOR_DETECT_VESC_CURRENT_TC_S;
	motor.ol_current_ki = s_detect_result.resistance_ohm /
		MOTOR_DETECT_VESC_CURRENT_TC_S;
	motor.ol_vq_limit = MOTOR_DETECT_VOLTAGE_LIMIT_FLUX_V;
	if (!Motor_Detect_RunStage(MOTOR_DETECT_STAGE_FLUX,
		flux_current_a, electrical_rpm, MOTOR_DETECT_TIMEOUT_FLUX_MS)) {
		PRINTF("Detect motor: flux invalid duty still=%.4f now=%.4f max=%.4f coast=%.1f eRPM\r\n",
			s_detect_control.flux_duty_still,
			s_detect_control.flux_duty_now,
			s_detect_control.flux_duty_max,
			s_detect_control.electrical_rpm_now);
		goto detect_done;
	}
	PRINTF("Detect motor: flux=%.4f mWb\r\n",
		s_detect_result.flux_linkage_wb * 1000.0f);
	PRINTF("Detect motor: coast |Bemf|=%.4fV at %.1f eRPM, Ke=%.3f V/krpm line-line RMS (%.0f pole pairs)\r\n",
		s_detect_control.resistance_voltage_avg,
		s_detect_control.electrical_rpm_now,
		128.25498f * motor.ol_pole_pairs * s_detect_result.flux_linkage_wb,
		motor.ol_pole_pairs);
	PRINTF("Detect motor: flux duty still=%.4f run=%.4f max=%.4f\r\n",
		s_detect_control.flux_duty_still,
		s_detect_control.flux_duty_now,
		s_detect_control.flux_duty_max);
	PRINTF("Detect motor: if you can measure the mechanical RPM now, run:\r\n");
	PRINTF("  detect poles %.0f <measured_mechanical_rpm>\r\n",
		electrical_rpm);
	PRINTF("Otherwise check the motor datasheet or count rotor magnets.\r\n");

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
	if (fabsf(electrical_rpm) < 1.0f || fabsf(mechanical_rpm) < 1.0f) {
		s_detect_result.error = MOTOR_DETECT_ERROR_MECHANICAL_REFERENCE;
		return false;
	}

	float ratio = fabsf(electrical_rpm / mechanical_rpm);
	float rounded = floorf(ratio + 0.5f);
	if (rounded < MOTOR_DETECT_POLE_PAIRS_MIN ||
		rounded > MOTOR_DETECT_POLE_PAIRS_MAX ||
		fabsf(ratio - rounded) > rounded * MOTOR_DETECT_POLE_PAIRS_RATIO_TOL) {
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

/* ---- Flash 持久化 ---- */

static bool Motor_Detect_ResultParametersValid(const motor_detect_result_t *result)
{
	return result->version == MOTOR_DETECT_RESULT_VERSION &&
		result->resistance_valid &&
		result->resistance_ohm >= MOTOR_DETECT_RESISTANCE_MIN_OHM &&
		result->resistance_ohm <= MOTOR_DETECT_RESISTANCE_MAX_OHM &&
		result->inductance_valid &&
		result->inductance_h >= MOTOR_DETECT_INDUCTANCE_MIN_H &&
		result->inductance_h <= MOTOR_DETECT_INDUCTANCE_MAX_H &&
		result->flux_valid &&
		result->flux_linkage_wb >= MOTOR_DETECT_FLUX_MIN_WB &&
		result->flux_linkage_wb <= MOTOR_DETECT_FLUX_MAX_WB;
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
		if (!FlashStorage_Save(&motor, &s_detect_result)) {
			return false;
		}
	}
	return true;
}

/**
 * @description: 从已加载的 Flash 数据中恢复检测结果（替代原 EasyFlash 读取版本）
 * @param stored: FlashStorage 加载时得到的 detect 数据（可为 NULL）
 */
void Motor_Detect_ApplyStoredResult(const motor_detect_result_t *stored)
{
	if (stored == NULL) {
		return;
	}

	if (Motor_Detect_ResultParametersValid(stored)) {
		s_detect_result = *stored;
		Motor_Detect_ApplyRuntime(&s_detect_result);
		PRINTF("Loaded detected motor: R=%.6f L=%.3fuH flux=%.4fmWb\r\n",
			s_detect_result.resistance_ohm,
			s_detect_result.inductance_h * 1000000.0f,
			s_detect_result.flux_linkage_wb * 1000.0f);
	}
}

/* ---- 错误名与中止 ---- */

const char *DETECT_GetErrorName(motor_detect_error_t error)
{
	switch (error) {
		case MOTOR_DETECT_ERROR_NONE:                 return "NONE";
		case MOTOR_DETECT_ERROR_BUSY:                 return "BUSY";
		case MOTOR_DETECT_ERROR_MOTOR_RUNNING:         return "MOTOR_RUNNING";
		case MOTOR_DETECT_ERROR_FAULT:                return "FAULT";
		case MOTOR_DETECT_ERROR_TIMEOUT:              return "TIMEOUT";
		case MOTOR_DETECT_ERROR_CURRENT:              return "CURRENT_TOO_HIGH";
		case MOTOR_DETECT_ERROR_RESISTANCE:           return "RESISTANCE_INVALID";
		case MOTOR_DETECT_ERROR_INDUCTANCE:           return "INDUCTANCE_INVALID";
		case MOTOR_DETECT_ERROR_FLUX:                 return "FLUX_INVALID";
		case MOTOR_DETECT_ERROR_MECHANICAL_REFERENCE:  return "MECHANICAL_REFERENCE_INVALID";
		case MOTOR_DETECT_ERROR_USER_ABORT:           return "USER_ABORT";
		default:                                      return "UNKNOWN";
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

/* ---- 霍尔检测 ---- */

/**
 * @description: 检测霍尔
 */
VOID DETECT_Hall(VOID)
{
	motor_all_state_t *motor = gpstMotor1;

	uint8_t hall_tab[8];
	bool res;
	mcpwm_foc_hall_detect(6.0 / 3.0, hall_tab, &res);

	if (res) {
		INFO("DETECT HALL Success.\r\n");
		for (int i = 0; i < 8; i++) {
			motor->m_conf->foc_hall_table[i] = hall_tab[i];
			PRINTF("%d ", hall_tab[i]);
		}
		PRINTF("\r\n");
		for (int i = 0; i < 6; i++) {
			PRINTF("%d ", gpstMotor->hallABC[i]);
		}
		PRINTF("\r\n");
	}
}

/* ---- 上电初始化 ---- */

/**
 * @description: 检测初始化
 * @detail:
 * 注意如果先检测,adc检测中断中处理foc的函数一定要关闭
 */
VOID Detect_Init(VOID)
{
	DETECT_ClearAbortRequest();
	DETECT_Offset(TRUE);
}

#ifdef __cplusplus
}
#endif
