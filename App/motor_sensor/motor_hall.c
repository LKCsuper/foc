/*
 * @Description: 三相霍尔读取与霍尔表标定
 * @Author: lkc
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
 * @description: 读取三相霍尔状态（bit0=A, bit1=B, bit2=C）
 * @param {bool} is_second_motor  true 时读第二路霍尔口
 * @param {int} samples           额外滤波次数，0 表示只采一次
 * @return {int} 霍尔状态 0~7
 * @author: lkc
 */
int Motor_Hall_Read(bool is_second_motor, int samples)
{
    int h1 = 0;
    int h2 = 0;
    int h3 = 0;
    int n;
    int tres;

    n = 1 + 2 * samples;
    tres = n / 2;

    if (is_second_motor) {
        while (n--) {
            h1 += READ_HALL1_2();
            h2 += READ_HALL2_2();
            h3 += READ_HALL3_2();
        }
    } else {
        while (n--) {
            h1 += READ_HALL1();
            h2 += READ_HALL2();
            h3 += READ_HALL3();
        }
    }

    return (h1 > tres) | ((h2 > tres) << 1) | ((h3 > tres) << 2);
}

/**
 * @description: 开环转一圈，标定霍尔状态与电角度对应表
 * @detail: 先用 D 轴电流吸住转子，再正反各转 3 圈采样霍尔，填 foc_hall_table。
 * @param {float} current        对齐电流 (A)
 * @param {uint8_t} *hall_table  输出 8 项霍尔表
 * @param {bool} *result         true 表示标定成功（6 个有效扇区）
 * @return {int} 故障码
 * @author: lkc
 */
int Motor_Hall_Detect(float current, uint8_t *hall_table, bool *result)
{
    motor_all_state_t *motor = gpstMotor1;
    int fault = FAULT_CODE_NONE;
    float sin_hall[8];
    float cos_hall[8];
    int hall_iterations[8];
    int i;
    int j;
    int hall;
    int fails;
    int z = 0;
    float s;
    float c;
    float ang;

    PRINTF("HALL DETECT\r\n");
    motor->m_phase_override = true;
    motor->m_id_set = 0.0f;
    motor->m_iq_set = 0.0f;
    motor->m_control_mode = CONTROL_MODE_CURRENT;
    motor->m_motor_released = false;
    motor->m_state = MC_STATE_RUNNING;
    Bsp_Tim_StartPwm(MOTOR1);

    motor->m_phase_now_override = 0.0f;
    *result = false;

    for (i = 0; i < 1000; i++) {
        motor->m_id_set = (float)i * current / 1000.0f;
        osDelay(pdMS_TO_TICKS(1));
    }

    memset(sin_hall, 0, sizeof(sin_hall));
    memset(cos_hall, 0, sizeof(cos_hall));
    memset(hall_iterations, 0, sizeof(hall_iterations));

    for (i = 0; i < 3; i++) {
        for (j = 0; j < 360; j++) {
            motor->m_phase_now_override = DEG2RAD_f((float)j);
            osDelay(pdMS_TO_TICKS(5));

            hall = Motor_Hall_Read(motor != &m_motor_1, motor->m_conf->m_hall_extra_samples);
            gpstMotor->curHallABC = (uint8_t)hall;
            if ((gpstMotor->curHallABC != gpstMotor->lastHallABC) && (z < 6)) {
                gpstMotor->lastHallABC = gpstMotor->curHallABC;
                gpstMotor->hallABC[z] = (uint8_t)hall;
                PRINTF("diff [%d] [%d] [%d] [%d]\r\n", z, gpstMotor->hallABC[z], gpstMotor->curHallABC, hall);
                z++;
            }

            arm_sin_cos_f32(motor->m_phase_now_override, &s, &c);
            sin_hall[hall] += s;
            cos_hall[hall] += c;
            hall_iterations[hall]++;
        }
    }

    for (i = 0; i < 3; i++) {
        for (j = 360; j >= 0; j--) {
            motor->m_phase_now_override = DEG2RAD_f((float)j);
            osDelay(pdMS_TO_TICKS(5));

            hall = Motor_Hall_Read(motor != &m_motor_1, motor->m_conf->m_hall_extra_samples);
            arm_sin_cos_f32(motor->m_phase_now_override, &s, &c);
            sin_hall[hall] += s;
            cos_hall[hall] += c;
            hall_iterations[hall]++;
        }
    }

    fails = 0;
    for (i = 0; i < 8; i++) {
        if (hall_iterations[i] > 30) {
            ang = RAD2DEG_f(atan2f(sin_hall[i], cos_hall[i]));
            utils_norm_angle(&ang);
            hall_table[i] = (uint8_t)(ang * 200.0f / 360.0f);
        } else {
            hall_table[i] = 255U;
            fails++;
        }
    }
    *result = (fails == 2);

    motor->m_id_set = 0.0f;
    motor->m_iq_set = 0.0f;
    motor->m_phase_override = false;
    motor->m_control_mode = CONTROL_MODE_NONE;
    motor->m_state = MC_STATE_OFF;
    Bsp_Tim_StopPwm(MOTOR1);

    return fault;
}

#ifdef __cplusplus
}
#endif
