/*
 * @Description: 
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2023-08-28 19:31:19
 * @FilePath: \foc\App\motor\utils.c
 * 
 * 几个任务
 * THD_FUNCTION 超时任务 timeout
 * 
 * 
 * 
 * foc任务
 * THD_FUNCTION 定时器任务
 * THD_FUNCTION 高频注入
 * THD_FUNCTION pid控制
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private typedef -----------------------------------------------------------*/
static volatile bool timer_thd_stop;
static volatile bool hfi_thd_stop;
static volatile bool pid_thd_stop;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: pid 控制
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Motor_Task_Pid(VOID)
{
    // (void)arg;

	// chRegSetThreadName("foc pid");

    // TODO 获取当前时间
	// uint32_t last_time = timer_time_now();

	for(;;) {
        // 定时器停止
		if (pid_thd_stop) {
			pid_thd_stop = false;
			return;
		}

        // pid 调度的频率
		// switch (m_motor_1.m_conf->sp_pid_loop_rate) {
		// case PID_RATE_25_HZ: chThdSleepMicroseconds(1000000 / 25); break;
		// case PID_RATE_50_HZ: chThdSleepMicroseconds(1000000 / 50); break;
		// case PID_RATE_100_HZ: chThdSleepMicroseconds(1000000 / 100); break;
		// case PID_RATE_250_HZ: chThdSleepMicroseconds(1000000 / 250); break;
		// case PID_RATE_500_HZ: chThdSleepMicroseconds(1000000 / 500); break;
		// case PID_RATE_1000_HZ: chThdSleepMicroseconds(1000000 / 1000); break;
		// case PID_RATE_2500_HZ: chThdSleepMicroseconds(1000000 / 2500); break;
		// case PID_RATE_5000_HZ: chThdSleepMicroseconds(1000000 / 5000); break;
		// case PID_RATE_10000_HZ: chThdSleepMicroseconds(1000000 / 10000); break;
		// }

        // TODO 获取调度时间差
		// float dt = timer_seconds_elapsed_since(last_time);
		// last_time = timer_time_now();

        // TODO 位置环 速度环
		//foc_run_pid_control_pos(encoder_index_found(), dt, (motor_all_state_t*)&m_motor_1);
		//foc_run_pid_control_speed(dt, (motor_all_state_t*)&m_motor_1);
#ifdef HW_HAS_DUAL_MOTORS
		foc_run_pid_control_pos(encoder_index_found(), dt, (motor_all_state_t*)&m_motor_2);
		foc_run_pid_control_speed(dt, (motor_all_state_t*)&m_motor_2);
#endif
	}
    
}

/**
 * @description: 高频注入
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Motor_Task_Hfi(VOID)
{
    // (void)arg;

	//chRegSetThreadName("foc hfi");

    // TODO 获取当前的时间
	//uint32_t t_last = timer_time_now();

	for(;;) {
		if (hfi_thd_stop) {
			hfi_thd_stop = false;
			return;
		}

        /* 当前时间差 */
		//float dt = timer_seconds_elapsed_since(t_last);
		/* 获取当前时间 */
        //t_last = timer_time_now();

        // 高频注入
		//hfi_update(&m_motor_1, dt);
// 双电机
#ifdef HW_HAS_DUAL_MOTORS
		hfi_update(&m_motor_2, dt);
#endif

        // 延时500 us
		// chThdSleepMicroseconds(500);
        /* 延时1ms */
        osDelay(pdMS_TO_TICKS(1));
	}

}

/**
 * @description: 定时器
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Motor_Task_Tim(VOID)
{
    // (void)arg;

	//chRegSetThreadName("foc timer");

	for(;;) {
		const float dt = 0.001;

        /* 线程是否停止 */
		if (timer_thd_stop) {
			timer_thd_stop = false;
			return;
		}

        // TODO 更新定时器
		//timer_update((motor_all_state_t*)&m_motor_1, dt);
/* 双电机 */
#ifdef HW_HAS_DUAL_MOTORS
		timer_update((motor_all_state_t*)&m_motor_2, dt);
#endif

        // 输入电流偏移测量
		// input_current_offset_measurement();

		//chThdSleepMilliseconds(1);
        /* 延时1ms */
        osDelay(pdMS_TO_TICKS(1));
	}

}

/**
 * @description: 任务初始化
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Motor_Task_Init(VOID)
{
    Motor_Task_Tim();
    Motor_Task_Hfi();
    Motor_Task_Pid();

    return;
}

#ifdef __cplusplus
}
#endif
