#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_sensor_update.h"
#include "motor_protect.h"

static unsigned s_encoder_updates;
static float s_encoder_dt;
static bool s_encoder_fault;
static bool s_detect_active;
static bool s_learning;
static bool s_fault_latched;
static unsigned s_fault_triggers;
static mc_fault_code s_fault_code;
static unsigned s_hall_reads;
static unsigned s_hall_updates;
static int s_hall_state = 5;
static const int8_t *s_hall_table;
static float s_hall_poles;
static float s_hall_dt;

void Motor_Encoder_Update(float dt_s)
{
    s_encoder_updates++;
    s_encoder_dt = dt_s;
}
bool Motor_Encoder_IsSignalFault(void) { return s_encoder_fault; }
bool Motor_Encoder_DetectIsActive(void) { return s_detect_active; }
bool Motor_Anticogging_IsLearning(void) { return s_learning; }
bool Motor_Protect_IsFaultLatched(void) { return s_fault_latched; }
void Motor_Protect_TriggerFault(mc_fault_code fault, const MOTOR_PARAM_S *motor)
{
    (void)motor;
    s_fault_triggers++;
    s_fault_code = fault;
}
int Motor_Hall_Read(bool second_motor, int samples)
{
    assert(!second_motor);
    assert(samples == 3);
    s_hall_reads++;
    return s_hall_state;
}
void Motor_Hall_AngleUpdateDefault(const int8_t *table, int hall_state, float poles, float dt_s)
{
    s_hall_updates++;
    s_hall_table = table;
    assert(hall_state == s_hall_state);
    s_hall_poles = poles;
    s_hall_dt = dt_s;
}

int main(void)
{
    MOTOR_PARAM_S motor;
    unsigned i;

    memset(&motor, 0, sizeof(motor));
    motor.ol_enable = true;
    motor.ol_pole_pairs = 7.0f;
    motor.hall_extra_samples = 3;

    s_encoder_fault = true;
    Motor_SensorUpdate_Step(&motor, MOTOR_FEEDBACK_SRC_ENCODER, 0.002f);
    assert(s_encoder_updates == 1U && fabsf(s_encoder_dt - 0.002f) < 1.0e-9f);
    assert(s_fault_triggers == 1U && s_fault_code == FAULT_CODE_ENCODER_SPI);

    s_detect_active = true;
    Motor_SensorUpdate_Step(&motor, MOTOR_FEEDBACK_SRC_ENCODER, 0.002f);
    assert(s_fault_triggers == 1U);
    s_detect_active = false;
    s_learning = true;
    Motor_SensorUpdate_Step(&motor, MOTOR_FEEDBACK_SRC_ENCODER, 0.002f);
    assert(s_fault_triggers == 1U);

    s_learning = false;
    s_encoder_fault = false;
    for (i = 0U; i < 3U; i++) {
        Motor_SensorUpdate_Step(&motor, MOTOR_FEEDBACK_SRC_HALL, 0.001f);
    }
    assert(s_hall_reads == 0U && s_hall_updates == 0U);
    Motor_SensorUpdate_Step(&motor, MOTOR_FEEDBACK_SRC_HALL, 0.001f);
    assert(s_hall_reads == 1U && s_hall_updates == 1U);
    assert(s_hall_table == motor.hall_table && s_hall_poles == 7.0f);
    assert(fabsf(s_hall_dt - 0.004f) < 1.0e-9f);

    Motor_SensorUpdate_Step(&motor, MOTOR_FEEDBACK_SRC_OPENLOOP, 0.001f);
    Motor_SensorUpdate_Step(&motor, MOTOR_FEEDBACK_SRC_HALL, 0.001f);
    assert(s_hall_updates == 1U);

    puts("test_motor_sensor_update: pass");
    return 0;
}
