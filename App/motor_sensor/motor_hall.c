/*
 * @Description: 
 * @Version: 1.0
 * @Author: lkc
 * @Date: 2023-10-02 22:16:11
 * @LastEditors: lkc
 * @LastEditTime: 2023-10-14 17:50:52
 * @detail: 
 * 
 * 使用
 * foc_correct_hall(motor_now->m_phase_now_observer, dt, motor_now, utils_read_hall(motor_now != &m_motor_1, conf_now->m_hall_extra_samples));
 * 
 * 霍尔如何检测
 * 
 * 
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// BLDC hall sensor table
#ifndef MCCONF_HALL_TAB_0
#define MCCONF_HALL_TAB_0				-1
#endif
#ifndef MCCONF_HALL_TAB_1
#define MCCONF_HALL_TAB_1				1
#endif
#ifndef MCCONF_HALL_TAB_2
#define MCCONF_HALL_TAB_2				3
#endif
#ifndef MCCONF_HALL_TAB_3
#define MCCONF_HALL_TAB_3				2
#endif
#ifndef MCCONF_HALL_TAB_4
#define MCCONF_HALL_TAB_4				5
#endif
#ifndef MCCONF_HALL_TAB_5
#define MCCONF_HALL_TAB_5				6
#endif
#ifndef MCCONF_HALL_TAB_6
#define MCCONF_HALL_TAB_6				4
#endif
#ifndef MCCONF_HALL_TAB_7
#define MCCONF_HALL_TAB_7				-1
#endif
#ifndef MCCONF_HALL_ERPM
#define MCCONF_HALL_ERPM				2000.0	// ERPM above which sensorless commutation is used in hybrid mode
#endif
#ifndef MCCONF_FOC_HALL_INTERP_ERPM
#define MCCONF_FOC_HALL_INTERP_ERPM		500		// Do not interpolate hall sensors below this ERPM
#endif
#ifndef MCCONF_M_HALL_EXTRA_SAMPLES
#define MCCONF_M_HALL_EXTRA_SAMPLES		1 // Extra samples for filtering when reading hall sensors
#endif

u8 hall_read_temp;
float hall_angle;
float hall_angle_add;
float hall_speed;


static int hall_to_phase_table[16];
static volatile int hall_detect_table[8][7];
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 霍尔角度计算
 * @detail: 平均速度估算
 * @return {*}
 * @author: lkc
 */
VOID MOTOR_HALL_AvgAngleEst(VOID)
{
	float avgSpeed = 0.0f;              /* 上个扇区速度 */
	float sectorTim = 0.0f;             /* 上个扇区使用时间 */
    float lastSectAngle = 0.0f;         /* 上一个扇区角度 */ 
    float curAngle = 0.0f;              /* 当前角度 */
    float dt = 0.0f;                    /* 时间间隔 */
	/* 计算原理 
	利用上个扇区的速度估算当前扇区的位置
	*/

	/* 平均角速度 = 60度（区间角度） / 上个扇区时间 */
	avgSpeed = (PI / 3) / sectorTim;

    curAngle = lastSectAngle + avgSpeed * dt;

	return;
}

/**
 * @description: 加速度估算
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID MOTOR_HALL_AccAngleEst(VOID)
{
	float avgSpeed = 0.0f;              /* 上个扇区速度 */
	float sectorTim = 0.0f;             /* 上个扇区使用时间 */
    float lastSectAngle = 0.0f;         /* 上一个扇区角度 */ 
    float curAngle = 0.0f;              /* 当前角度 */
    float curSpeed = 0.0f;              /* 当前角度 */
    float dt = 0.0f;                    /* 时间间隔 */

    float avgSect1Speed = 0.0f;         /* 片区1速度 */
    float avgSect2Speed = 0.0f;         /* 片区2速度 */

    float sectorTim1 = 0.0f;            /* 片区1使用时间 */
    float sectorTim2 = 0.0f;            /* 片区2使用时间 */
    float acc = 0.0f;                   /* 加速度 */

    acc = 2 * (avgSect2Speed - avgSect1Speed) / (sectorTim1 + sectorTim2);
    curSpeed = avgSect2Speed + acc * dt;
    curAngle = lastSectAngle + avgSect2Speed * dt + 1/2 * acc * dt * dt;

	return;
}

/**
 * @description: 锁相环估算
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID MOTOR_HALL_PllEst(VOID)
{

    return;
}



/**
 * Read hall sensors
 * 读取霍尔传感器
 * @param is_second_motor
 * Use hall sensor port for second motor on dual motor hardware.
 *
 * @param samples
 * is_second_motor 是否是二电机
 * samples MCCONF_M_HALL_EXTRA_SAMPLES 采样过滤， 默认采样一次
 * The number of extra samples to read and filter over. If this
 * is 0, only one sample will be used.
 *
 * @return
 * 三个霍尔传感器的状态
 * The state of the three hall sensors.
 */
int utils_read_hall(bool is_second_motor, int samples) {
	samples = 1 + 2 * samples;

	int h1 = 0, h2 = 0, h3 = 0;
	int tres = samples / 2;

	if (is_second_motor) {
		while (samples--) {
			h1 += READ_HALL1_2();
			h2 += READ_HALL2_2();
			h3 += READ_HALL3_2();
		}
	} else {
		while (samples--) {
			h1 += READ_HALL1();
			h2 += READ_HALL2();
			h3 += READ_HALL3();
		}
	}

	return (h1 > tres) | ((h2 > tres) << 1) | ((h3 > tres) << 2);
}

/**
 * 检测霍尔
 * Run the motor in open loop and figure out at which angles the hall sensors are.
 *
 * @param current (电流大小，检测最大电流 / 3)
 * Current to use.
 *
 * @param hall_table
 * Table to store the result to.
 *
 * @result
 * true: Success
 * false: Something went wrong
 *
 * @return
 * The fault code
 */
int mcpwm_foc_hall_detect(float current, uint8_t *hall_table, bool *result) {
	//volatile motor_all_state_t *motor = get_motor_now();
	motor_all_state_t *motor = gpstMotor1;
	
	int fault = FAULT_CODE_NONE;
	//mc_interface_lock();
	PRINTF("HALL DETECT\r\n");
	motor->m_phase_override = true;
	motor->m_id_set = 0.0;
	motor->m_iq_set = 0.0;
	motor->m_control_mode = CONTROL_MODE_CURRENT;
	motor->m_motor_released = false;
	motor->m_state = MC_STATE_RUNNING;
	Bsp_Tim_StartPwm(MOTOR1);

	// MTPA overrides id target
	// MTPA_MODE mtpa_old = motor->m_conf->foc_mtpa_mode;
	// motor->m_conf->foc_mtpa_mode = MTPA_MODE_OFF;

	// Disable timeout
	// systime_t tout = timeout_get_timeout_msec();
	// float tout_c = timeout_get_brake_current();
	// KILL_SW_MODE tout_ksw = timeout_get_kill_sw_mode();
	// timeout_reset();
	// timeout_configure(60000, 0.0, KILL_SW_MODE_DISABLED);

	// Lock the motor
	motor->m_phase_now_override = 0;								/* 先设置角度0，锁住电机 */

	*result = false;												/* 结果false */

    /* d轴不断增大作用,我觉得可能是到0点 */
	for (int i = 0;i < 1000;i++) {
		motor->m_id_set = (float)i * current / 1000.0;
        // TODO 检测错误
		// fault = mc_interface_get_fault();
		// if (fault != FAULT_CODE_NONE) {
		// 	goto exit_hall_detect;
		// }
        osDelay(pdMS_TO_TICKS(1));
		//chThdSleepMilliseconds(1);
	}

	float sin_hall[8];
	float cos_hall[8];
	int hall_iterations[8];
	memset(sin_hall, 0, sizeof(sin_hall));
	memset(cos_hall, 0, sizeof(cos_hall));
	memset(hall_iterations, 0, sizeof(hall_iterations));

	// Forwards 向前转
	static int z = 0;
	for (int i = 0;i < 3;i++) {
		for (int j = 0;j < 360;j++) {
			motor->m_phase_now_override = DEG2RAD_f(j);
            // TODO 检测错误
			// fault = mc_interface_get_fault();
			// if (fault != FAULT_CODE_NONE) {
			// 	goto exit_hall_detect;
			// }
             osDelay(pdMS_TO_TICKS(5));
			//chThdSleepMilliseconds(5);

            // 读取霍尔值
			int hall = utils_read_hall(motor != &m_motor_1, motor->m_conf->m_hall_extra_samples);
			gpstMotor->curHallABC = hall;
			if ((gpstMotor->curHallABC != gpstMotor->lastHallABC) && (z < 6))
			{
				gpstMotor->lastHallABC = gpstMotor->curHallABC;
				gpstMotor->hallABC[z] = hall;
				PRINTF("diff [%d] [%d] [%d] [%d]\r\n", z, gpstMotor->hallABC[z], gpstMotor->curHallABC, hall);
				z++;
			}

			float s, c;
			//sincosf(motor->m_phase_now_override, &s, &c);
			arm_sin_cos_f32(motor->m_phase_now_override, &s, &c);
			sin_hall[hall] += s;
			cos_hall[hall] += c;
			hall_iterations[hall]++;
		}
	}

	// Reverse 向后转
	for (int i = 0;i < 3;i++) {
		for (int j = 360;j >= 0;j--) {
			motor->m_phase_now_override = DEG2RAD_f(j);
            // TODO 检测错误
			// fault = mc_interface_get_fault();
			// if (fault != FAULT_CODE_NONE) {
			// 	goto exit_hall_detect;
			// }
            osDelay(pdMS_TO_TICKS(5));
			//chThdSleepMilliseconds(5);

			int hall = utils_read_hall(motor != &m_motor_1, motor->m_conf->m_hall_extra_samples);
			//PRINTF("HALL2 [%d] \r\n", hall);
			float s, c;
			//sincosf(motor->m_phase_now_override, &s, &c);
			arm_sin_cos_f32(motor->m_phase_now_override, &s, &c);
			sin_hall[hall] += s;
			cos_hall[hall] += c;
			hall_iterations[hall]++;
		}
	}

	int fails = 0;
	for(int i = 0;i < 8;i++) {
		if (hall_iterations[i] > 30) {
			float ang = RAD2DEG_f(atan2f(sin_hall[i], cos_hall[i]));
			utils_norm_angle(&ang);
			hall_table[i] = (uint8_t)(ang * 200.0 / 360.0);
		} else {
			hall_table[i] = 255;
			fails++;
		}
	}
	*result = (fails == 2);

	exit_hall_detect:
	motor->m_id_set = 0.0;
	motor->m_iq_set = 0.0;
	motor->m_phase_override = false;
	motor->m_control_mode = CONTROL_MODE_NONE;
	motor->m_state = MC_STATE_OFF;
	Bsp_Tim_StopPwm(MOTOR1);
	//stop_pwm_hw((motor_all_state_t*)motor);
    
	// motor->m_conf->foc_mtpa_mode = mtpa_old;
	// timeout_configure(tout, tout_c, tout_ksw);
	// mc_interface_unlock();

	return fault;
}

/**
 * @description: 霍尔中断
 * @detail: 
 * @return {*}
 * @author: lkc
 */
void Motor_Hall_IRQHandler(void)
{
	#if 0
	float temp;
	if(TIM_GetFlagStatus(HALL_TIM, TIM_FLAG_CC1)==SET)
	{
		temp = (float)(TIM_GetCapture1(HALL_TIM));										/* 获取定时器count */
		hall_angle_add = (float)HALL_ANGLE_FACTOR/(float)(temp); // (上一个周期的速度 * 当前的时间)
		hall_speed = (float)HALL_SPEED_FACTOR/(float)(temp);
		/* 读取hall的值 */
		hall_read_temp = GPIO_ReadInputDataBit(HALL_CH3_GPIO_PORT,HALL_CH3_PIN);
		hall_read_temp |= GPIO_ReadInputDataBit(HALL_CH2_GPIO_PORT,HALL_CH2_PIN)<<1;
		hall_read_temp |= GPIO_ReadInputDataBit(HALL_CH1_GPIO_PORT,HALL_CH1_PIN)<<2;
		
		if(0x05 == hall_read_temp)			/* 101 */
		{
			hall_angle = 0.0f+PHASE_SHIFT_ANGLE;
		} 
		else if(0x04 == hall_read_temp)		/* 100 */
		{
			hall_angle = (PI/3.0f)+PHASE_SHIFT_ANGLE;
		}
		else if(0x06 == hall_read_temp)		/* 110 */
		{
			hall_angle = (PI*2.0f/3.0f)+PHASE_SHIFT_ANGLE;
		}
		else if(0x02 == hall_read_temp)		/* 010 */
		{
			hall_angle = PI+PHASE_SHIFT_ANGLE;
		}
		else if(0x03 == hall_read_temp)		/* 011 */
		{
			hall_angle = (PI*4.0f/3.0f)+PHASE_SHIFT_ANGLE;
		}
		else if(0x01 == hall_read_temp)		/* 001 */
		{
			hall_angle = (PI*5.0f/3.0f)+PHASE_SHIFT_ANGLE;
		}

		if(hall_angle < 0.0f)
		{
			hall_angle += 2.0f*PI;
		}
		else if(hall_angle > (2.0f*PI))
		{
			hall_angle -= 2.0f*PI;
		}
		
		TIM_ClearFlag(HALL_TIM, TIM_FLAG_CC1);
	}
  #endif
	return;
}
#if 0
	conf->hall_table[0] = MCCONF_HALL_TAB_0;
	conf->hall_table[1] = MCCONF_HALL_TAB_1;
	conf->hall_table[2] = MCCONF_HALL_TAB_2;
	conf->hall_table[3] = MCCONF_HALL_TAB_3;
	conf->hall_table[4] = MCCONF_HALL_TAB_4;
	conf->hall_table[5] = MCCONF_HALL_TAB_5;
	conf->hall_table[6] = MCCONF_HALL_TAB_6;
	conf->hall_table[7] = MCCONF_HALL_TAB_7;
	conf->hall_sl_erpm = MCCONF_HALL_ERPM;
	
/**
 * Reset the hall sensor detection table
 */
void mcpwm_reset_hall_detect_table(void) {
	memset((void*)hall_detect_table, 0, sizeof(hall_detect_table[0][0]) * 8 * 7);
}

float foc_correct_hall(float angle, float dt, motor_all_state_t *motor, int hall_val) 
{
	mc_configuration *conf_now = motor->m_conf;
	motor->m_hall_dt_diff_now += dt;

	float rad_per_sec = (M_PI / 3.0) / motor->m_hall_dt_diff_last;		/* 每秒弧度：2 * PI / 6 * 时间差 */
	float rpm_abs_fast = fabsf(RADPS2RPM_f(motor->m_speed_est_fast));	/* 转速绝对值 */
	float rpm_abs_hall = fabsf(RADPS2RPM_f(rad_per_sec));				/* 霍尔转速 每分钟 */

	// Hysteresis 5 % of total speed
	float hyst = conf_now->foc_sl_erpm * 0.1;
	if (motor->m_using_hall) {
		if (fminf(rpm_abs_fast, rpm_abs_hall) > (conf_now->foc_sl_erpm + hyst)) {
			motor->m_using_hall = false;
		}
	} else {
		if (rpm_abs_fast < (conf_now->foc_sl_erpm - hyst)) {
			motor->m_using_hall = true;
		}
	}

	int ang_hall_int = conf_now->foc_hall_table[hall_val];

	// Only override the observer if the hall sensor value is valid.
	if (ang_hall_int < 201) {
		// Scale to the circle and convert to radians
		float ang_hall_now = ((float)ang_hall_int / 200.0) * 2 * M_PI;

		if (motor->m_ang_hall_int_prev < 0) {
			// Previous angle not valid
			motor->m_ang_hall_int_prev = ang_hall_int;
			motor->m_ang_hall = ang_hall_now;
		} else if (ang_hall_int != motor->m_ang_hall_int_prev) {
			int diff = ang_hall_int - motor->m_ang_hall_int_prev;
			if (diff > 100) {
				diff -= 200;
			} else if (diff < -100) {
				diff += 200;
			}

			// This is only valid if the direction did not just change. If it did, we use the
			// last speed together with the sign right now.
			if (SIGN(diff) == SIGN(motor->m_hall_dt_diff_last)) {
				if (diff > 0) {
					motor->m_hall_dt_diff_last = motor->m_hall_dt_diff_now;
				} else {
					motor->m_hall_dt_diff_last = -motor->m_hall_dt_diff_now;
				}
			} else {
				motor->m_hall_dt_diff_last = -motor->m_hall_dt_diff_last;
			}

			motor->m_hall_dt_diff_now = 0.0;

			// A transition was just made. The angle is in the middle of the new and old angle.
			int ang_avg = motor->m_ang_hall_int_prev + diff / 2;
			ang_avg %= 200;

			// Scale to the circle and convert to radians
			motor->m_ang_hall = ((float)ang_avg / 200.0) * 2 * M_PI;
		}

		motor->m_ang_hall_int_prev = ang_hall_int;

		if (RADPS2RPM_f((M_PI / 3.0) /
				fmaxf(fabsf(motor->m_hall_dt_diff_now),
						fabsf(motor->m_hall_dt_diff_last))) < conf_now->foc_hall_interp_erpm) {
			// Don't interpolate on very low speed, just use the closest hall sensor. The reason is that we might
			// get stuck at 60 degrees off if a direction change happens between two steps.
			motor->m_ang_hall = ang_hall_now;
		} else {
			// Interpolate
			float diff = utils_angle_difference_rad(motor->m_ang_hall, ang_hall_now);
			if (fabsf(diff) < ((2.0 * M_PI) / 12.0)) {
				// Do interpolation
				motor->m_ang_hall += rad_per_sec * dt;
			} else {
				// We are too far away with the interpolation
				motor->m_ang_hall -= diff / 100.0;
			}
		}

		// Limit hall sensor rate of change. This will reduce current spikes in the current controllers when the angle estimation
		// changes fast.
		float angle_step = (fmaxf(rpm_abs_hall, conf_now->foc_hall_interp_erpm) / 60.0) * 2.0 * M_PI * dt * 1.5;
		float angle_diff = utils_angle_difference_rad(motor->m_ang_hall, motor->m_ang_hall_rate_limited);
		if (fabsf(angle_diff) < angle_step) {
			motor->m_ang_hall_rate_limited = motor->m_ang_hall;
		} else {
			motor->m_ang_hall_rate_limited += angle_step * SIGN(angle_diff);
		}

		utils_norm_angle_rad((float*)&motor->m_ang_hall_rate_limited);
		utils_norm_angle_rad((float*)&motor->m_ang_hall);

		if (motor->m_using_hall) {
			angle = motor->m_ang_hall_rate_limited;
		}
	} else {
		// Invalid hall reading. Don't update angle.
		motor->m_ang_hall_int_prev = -1;

		// Also allow open loop in order to behave like normal sensorless
		// operation. Then the motor works even if the hall sensor cable
		// gets disconnected (when the sensor spacing is 120 degrees).
		if (motor->m_phase_observer_override && motor->m_state == MC_STATE_RUNNING) {
			angle = motor->m_phase_now_observer_override;
		}
	}

	return angle;
}

/**
 * Initialize the hall sensor lookup table
 *
 * @param table
 * The commutations corresponding to the hall sensor states in the forward direction-
 */
void mcpwm_init_hall_table(int8_t *table) {
	const int fwd_to_rev[7] = {-1,1,6,5,4,3,2};

	for (int i = 0;i < 8;i++) {
		hall_to_phase_table[8 + i] = table[i];
		int ind_now = hall_to_phase_table[8 + i];

		if (ind_now < 1) {
			hall_to_phase_table[i] = ind_now;
			continue;
		}

		hall_to_phase_table[i] = fwd_to_rev[ind_now];
	}
}

/**
 * Reset the hall sensor detection table
 */
void mcpwm_reset_hall_detect_table(void) {
	memset((void*)hall_detect_table, 0, sizeof(hall_detect_table[0][0]) * 8 * 7);
}

/**
 * Get the current detected hall sensor table
 *
 * @param table
 * Pointer to a table where the result should be stored
 *
 * @return
 * 0: OK
 * -1: Invalid hall sensor output
 * -3: Encoder enabled
 */
int mcpwm_get_hall_detect_result(int8_t *table) {
	if (conf->m_sensor_port_mode != SENSOR_PORT_MODE_HALL) {
		return -3;
	}

	for (int i = 0;i < 8;i++) {
		int samples = 0;
		int res = -1;
		for (int j = 1;j < 7;j++) {
			if (hall_detect_table[i][j] > samples) {
				samples = hall_detect_table[i][j];
				if (samples > 15) {
					res = j;
				}
			}
			table[i] = res;
		}
	}

	int invalid_samp_num = 0;
	int nums[7] = {0, 0, 0, 0, 0, 0, 0};
	int tot_nums = 0;
	for (int i = 0;i < 8;i++) {
		if (table[i] == -1) {
			invalid_samp_num++;
		} else {
			if (!nums[table[i]]) {
				nums[table[i]] = 1;
				tot_nums++;
			}
		}
	}

	if (invalid_samp_num == 2 && tot_nums == 6) {
		return 0;
	} else {
		return -1;
	}
}
/**
 * Read the current phase of the motor using hall effect sensors
 * @return
 * The phase read.
 */
int mcpwm_read_hall_phase(void) {
	return hall_to_phase_table[read_hall() + (direction ? 8 : 0)];
}

static int read_hall(void) {
	return READ_HALL1() | (READ_HALL2() << 1) | (READ_HALL3() << 2);
}

#endif
#ifdef __cplusplus
}
#endif