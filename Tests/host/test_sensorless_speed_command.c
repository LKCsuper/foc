#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "motor.h"
#include "motor_sensorless.h"
#include <stdarg.h>
static char output[16384];
static unsigned output_size;
static void fake_printf(const char *format, ...) {
 va_list ap;va_start(ap,format);
 int n=vsnprintf(output+output_size,sizeof(output)-output_size,format,ap);
 va_end(ap);assert(n>=0 && (unsigned)n<sizeof(output)-output_size);output_size+=(unsigned)n;
}
#define PRINTF(...) fake_printf(__VA_ARGS__)
#define pdMS_TO_TICKS(x) (x)
MOTOR_PARAM_S motor;
static motor_config_t config;
static motor_command_t command;
static motor_feedback_source_t source;
static uint32_t clock_us, abort_us;
static bool fault, never_lock, lose_lock;
static unsigned stops;
static bool current_failure, read_failure;
static bool underspeed;
uint32_t Hal_Motor_PwmPeriodCounts(void) { return 5600; }
uint32_t Hal_Motor_TimeUs(void) { return clock_us; }
static void osDelay(uint32_t ms) { clock_us += ms * 1000U; }
static bool speed_stop_key(void) { return abort_us && clock_us >= abort_us; }
static bool ol_capture_config(motor_config_t *out) { *out=config; return true; }
motor_result_t Motor_GetCommand(const MOTOR_PARAM_S *m, motor_command_t *out) { (void)m; *out=command; return MOTOR_OK; }
motor_result_t Motor_SetCommand(MOTOR_PARAM_S *m, const motor_command_t *c)
{
    (void)m;
    if (c->enable && c->target_rpm > 800)
        assert(config.sensorless_takeover && c->speed_enable);
    if (c->enable)
        assert(fabsf(config.speed_iq_limit - (c->target_rpm > 1800 ? .30f : .15f)) < 1e-6f);
    command = *c;
    motor.ol_enable = c->enable;
    return MOTOR_OK;
}
motor_result_t Motor_GetStatus(const MOTOR_PARAM_S *m,motor_status_t *s) { (void)m; memset(s,0,sizeof(*s));s->enable=command.enable;s->target_rpm=command.target_rpm;s->iq=current_failure && clock_us>=1000000U ? 1.25f : .3f;return read_failure && clock_us>=1000000U ? (motor_result_t)-1 : MOTOR_OK; }
motor_result_t Motor_SetConfig(MOTOR_PARAM_S *m,const motor_config_t *c) { (void)m;config=*c;command.speed_enable=c->speed_enable;return MOTOR_OK; }
motor_result_t Motor_RequestStop(MOTOR_PARAM_S *m) { (void)m;command.enable=false;motor.ol_enable=false;stops++;return MOTOR_OK; }
motor_feedback_source_t Motor_GetFeedbackSource(void) { return source; }
motor_result_t Motor_SetFeedbackSource(motor_feedback_source_t s) { source=s;return MOTOR_OK; }
bool Motor_Bldc_IsSelected(void) { return false; }
bool Motor_Detect_IsActive(void) { return false; }
bool Motor_Protect_IsFaultLatched(void) { return fault; }
static bool Motor_Position_Enable(bool x) { (void)x;return true; }
static bool Motor_Joint_Enable(bool x) { (void)x;return true; }
void Motor_Timing_TraceArmEntry(void) {}
void Motor_Timing_TraceStop(void) {}
motor_sensorless_state_t Motor_Sensorless_GetState(void) { return config.sensorless_takeover && !never_lock && !(lose_lock && clock_us>12000000U) ? MOTOR_SENSORLESS_CLOSED : MOTOR_SENSORLESS_LOCKING; }
float Motor_Sensorless_GetMechanicalRpm(void) { return underspeed && command.target_rpm>=2200 ? .87f*command.target_rpm : command.target_rpm; }
bool Motor_SpeedLoop_IsFeedbackValid(void) { return command.speed_enable && Motor_Sensorless_GetState()==MOTOR_SENSORLESS_CLOSED; }
bool Motor_Encoder_IsCompareEnabled(void) { return false; }
bool Motor_Encoder_IsSpeedValid(void) { return false; }
float Motor_Encoder_GetSampleAge(void) { return 1; }
float Motor_Encoder_GetMechRpm(void) { return 0; }
#include "sltest_under_test.inc"
static void init(void) {
 memset(&motor,0,sizeof(motor));memset(&command,0,sizeof(command));memset(&config,0,sizeof(config));
 config.ol_max_rpm=1000;config.sensorless_transition_rpm=500;config.sensorless_fallback_rpm=300;
 config.ol_vq_limit=4;
 config.sensorless_motor_r=2.925f;config.sensorless_motor_l=.00086f;config.sensorless_flux_linkage=.0035f;
 motor.v_bus=16;motor.ol_pole_pairs=7;source=MOTOR_FEEDBACK_SRC_OPENLOOP;
 clock_us=abort_us=stops=output_size=0;output[0]=0;current_failure=read_failure=false;fault=never_lock=lose_lock=false;
 underspeed=false;
}
int main(void) {
 char *args[]={"sltest"}; motor_config_t saved;
 init();saved=config; assert(Cmd_SensorlessTest(1,args)==0);assert(stops==1 && !command.enable && !command.speed_enable && command.target_rpm==0);assert(memcmp(&saved,&config,sizeof(saved))==0 && source==MOTOR_FEEDBACK_SRC_OPENLOOP);
 init();never_lock=true;assert(Cmd_SensorlessTest(1,args)==-1 && stops==1 && !command.enable);
 init();lose_lock=true;assert(Cmd_SensorlessTest(1,args)==-1 && stops==1 && !command.enable);
 init();abort_us=1000000;assert(Cmd_SensorlessTest(1,args)==-1 && stops==1 && !command.enable);
 init();fault=true;assert(Cmd_SensorlessTest(1,args)==-1 && stops==0);
 init();char *bad[]={"sltest","nan","600"};assert(Cmd_SensorlessTest(3,bad)==-1 && stops==0);
 init();char *fast[]={"sltest","5","3000"};assert(Cmd_SensorlessTest(3,fast)==-1 && stops==0);
 init();char *high[]={"sltest","5","600","1000","1400","1800","2200","2600"};
 assert(Cmd_SensorlessTest(8,high)==-1 && stops==0); /* 原最大转速不足，不夹紧后假通过 */
 init();config.ol_max_rpm=6000;saved=config;
 assert(Cmd_SensorlessTest(8,high)==0 && stops==1);
 assert(memcmp(&saved,&config,sizeof(saved))==0 && !command.enable);
 init();current_failure=true;assert(Cmd_SensorlessTest(1,args)==-1 && stops==2 && !command.enable);
 assert(strstr(output,"reason=0x20") && strstr(output,"iq=1.2500") && strstr(output,"enable=1"));
 init();read_failure=true;assert(Cmd_SensorlessTest(1,args)==-1 && stops==2 && !command.enable);
 assert(strstr(output,"reason=0x01"));
 init();config.ol_max_rpm=6000;saved=config;underspeed=true;
 assert(Cmd_SensorlessTest(8,high)==-1 && stops==1 && !command.enable);
 assert(strstr(output,"speed not tracking") && memcmp(&saved,&config,sizeof(saved))==0);
 puts("sensorless speed command: pass");return 0;
}
