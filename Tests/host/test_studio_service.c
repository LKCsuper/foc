#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "motor.h"
#include "studio_protocol.h"
#include "cmd_studio.h"
#define PRINTF(...) ((void)0)
typedef unsigned long ULONG;
MOTOR_PARAM_S motor;
static uint32_t test_now;
static unsigned stops, starts, writes, config_sets;
static bool other_control, transport_full;
static uint8_t last_packet[96];
uint32_t Motor_Crit_Enter(void) { return 0; }
void Motor_Crit_Exit(uint32_t key) { (void)key; }
uint32_t Hal_Motor_TimeUs(void) { return test_now; }
int32_t osKernelLock(void) { return 0; }
int32_t osKernelRestoreLock(int32_t lock) { return lock; }
bool Motor_Position_IsEnabled(void) { return other_control; }
bool Motor_Joint_IsEnabled(void) { return false; }
bool Motor_Bldc_IsSelected(void) { return false; }
bool Motor_Detect_IsActive(void) { return false; }
bool Motor_Hall_DetectIsActive(void) { return false; }
bool Motor_Protect_IsFaultLatched(void) { return false; }
bool Cmd_SerialPlotEnsureTask(void) { return true; }
bool Bsp_Usart_TryWriteFrame(const uint8_t *data, ULONG n) { if(transport_full)return false;memcpy(last_packet,data,n);writes++;return true; }
motor_result_t Motor_GetCommand(const MOTOR_PARAM_S *m,motor_command_t *c) { memset(c,0,sizeof(*c));c->enable=m->ol_enable;c->mode=m->ol_mode;c->target_rpm=m->ol_target_rpm;c->iq_set=m->ol_iq_set;c->id_set=m->ol_id_set;return MOTOR_OK; }
motor_result_t Motor_SetCommand(MOTOR_PARAM_S *m,const motor_command_t *c) { if(c->enable&&!m->ol_enable)starts++;m->ol_enable=c->enable;m->ol_target_rpm=c->target_rpm;m->ol_iq_set=c->iq_set;m->ol_id_set=c->id_set;return MOTOR_OK; }
motor_result_t Motor_RequestStop(MOTOR_PARAM_S *m) { m->ol_enable=false;stops++;return MOTOR_OK; }
motor_result_t Motor_GetStatus(const MOTOR_PARAM_S *m,motor_status_t *s) { memset(s,0,sizeof(*s));s->enable=m->ol_enable;s->mode=m->ol_mode;s->target_rpm=m->ol_target_rpm;return MOTOR_OK; }
bool Motor_Settings_Capture(const MOTOR_PARAM_S *m,motor_settings_t *s) { memset(s,0,sizeof(*s));s->config.speed_iq_limit=m->speed_iq_limit;return true; }
motor_result_t Motor_SetConfig(MOTOR_PARAM_S *m,const motor_config_t *c) { config_sets++;m->speed_kp=c->speed_kp;return MOTOR_OK; }
/* 由测试入口从真实 cmd_studio.c 提取，不复制实现。 */
#include "studio_service_under_test.inc"
static int command(const char *sub,const char *a,const char *b)
{
    char *argv[4]={(char *)"studio",(char *)sub,(char *)a,(char *)b};
    return Cmd_Studio(b?4:a?3:2,argv);
}
static void target(unsigned sequence,const char *rpm,const char *iq)
{
    char body[128],line[160];
    sprintf(body,"@C1,%lu,%u,%s,%s,0",(unsigned long)s_lease.token,sequence,rpm,iq);
    sprintf(line,"%s*%04X",body,Studio_Crc16((const uint8_t *)body,strlen(body)));
    Cmd_StudioTargetLine(line);
}
int main(void)
{
    assert(sizeof(s_fields)/sizeof(s_fields[0])==63);
    motor.ol_mode=1;motor.ol_max_rpm=6000;motor.speed_iq_limit=2;
    assert(command("on",NULL,NULL)==0);assert(starts==0);
    assert(command("control","1234",NULL)==0);assert(!motor.ol_enable);
    assert(command("start",NULL,NULL)!=0);
    test_now=20000;target(1,"100","0.5");Cmd_StudioPoll();
    assert(motor.ol_target_rpm==100&&!motor.ol_enable&&s_applied_sequence==1);
    assert(command("start",NULL,NULL)==0&&motor.ol_enable&&starts==1);
    target(1,"200","0.5");Cmd_StudioPoll();assert(motor.ol_target_rpm==100);
    target(2,"200","3");Cmd_StudioPoll();assert(s_applied_sequence==1);
    transport_full=true;test_now=320000;Cmd_StudioPoll();
    assert(!motor.ol_enable&&!Cmd_StudioOwnsControl()&&stops>0);
    target(3,"100","0.5");Cmd_StudioPoll();assert(!motor.ol_enable&&starts==1);
    transport_full=false;assert(command("control","5678",NULL)==0);
    test_now+=20000;target(1,"100","0.5");Cmd_StudioPoll();
    assert(command("release",NULL,NULL)==0);assert(command("start",NULL,NULL)!=0);
    target(2,"500","1");Cmd_StudioPoll();assert(!motor.ol_enable);
    assert(command("set","5","15000")!=0);assert(config_sets==0); /* 采样基准只读 */
    assert(command("set","21","0.004")==0);assert(config_sets==1);
    motor.ol_enable=true;assert(command("set","21","0.005")!=0);assert(config_sets==1);
    motor.ol_enable=false;other_control=true;assert(command("control","9",NULL)!=0);
    other_control=false;assert(command("control","4294967296",NULL)!=0);assert(command("control","-1",NULL)!=0);
    motor.ol_enable=true;assert(command("release",NULL,NULL)==0);assert(!motor.ol_enable);
    assert(writes>0);puts("studio service: explicit start, replay, budget, full TX timeout, release and config guards PASS");
    return 0;
}
