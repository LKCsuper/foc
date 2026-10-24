/* 网页调试：50 Hz 二进制遥测、配置回读、带租约的目标交接。
 * 控制帧在 Cmd_Task 校验并放入单槽邮箱，Plot 任务应用目标并检查 300 ms 超时。
 * 不增设 ISR 工作，不修改 ADC/PWM/定时器配置；不自动启动、不自动写 Flash。 */
#include "includes.h"
#include "cmd_studio.h"
#include "studio_protocol.h"
#include "motor_crit.h"
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#if MOTOR_CFG_CMD
typedef struct {
    uint16_t motor_offset;
    uint16_t config_offset;
    uint8_t type; /* 0=float, 1=bool, 2=uint16, 3=int, 4=int8 */
    uint8_t writable;
} studio_field_t;
#define FIELD(name, kind, write) {offsetof(MOTOR_PARAM_S, name), offsetof(motor_config_t, name), kind, write}
#define CAL(name, index, kind, size) {offsetof(MOTOR_PARAM_S, name) + (index) * (size), 0, kind, 0}
static const studio_field_t s_fields[] = {
#include "studio_fields.inc"
};
#undef FIELD
#undef CAL
static volatile bool s_active;
static studio_lease_t s_lease;
static studio_target_t s_pending_target;
static bool s_pending;
static bool s_stop_requested;
static volatile uint16_t s_param_cursor = STUDIO_PARAM_COUNT;
static uint32_t s_publish_us;
static uint32_t s_applied_sequence;
static int32_t s_result;
static uint16_t s_packet_sequence;

static void put32(uint8_t *p, uint32_t value) { p[0]=(uint8_t)value; p[1]=(uint8_t)(value>>8); p[2]=(uint8_t)(value>>16); p[3]=(uint8_t)(value>>24); }
static void putfloat(uint8_t *p, float value) { uint32_t bits; memcpy(&bits,&value,4); put32(p,bits); }
static bool send_packet(uint8_t type, const uint8_t *data, uint16_t length)
{
    uint8_t output[96];
    uint16_t sequence;
    uint32_t key=Motor_Crit_Enter();
    size_t n;
    sequence=s_packet_sequence++;
    Motor_Crit_Exit(key);
    n = Studio_Encode(type, sequence, data, length, output);
    return n != 0U && Bsp_Usart_TryWriteFrame(output, (ULONG)n);
}
static float field_value(unsigned index)
{
    float value;
    uint32_t key = Motor_Crit_Enter();
    const studio_field_t *field = &s_fields[index];
    const uint8_t *p = (const uint8_t *)&motor + field->motor_offset;
    if (field->type == 1U) value = *(const bool *)p ? 1.0f : 0.0f;
    else if (field->type == 2U) value = (float)*(const uint16_t *)p;
    else if (field->type == 3U) value = (float)*(const int *)p;
    else if (field->type == 4U) value = (float)(uint8_t)*(const int8_t *)p;
    else memcpy(&value,p,sizeof(value));
    Motor_Crit_Exit(key);
    return value;
}
static void acknowledge(uint16_t id, int result, float value)
{
    uint8_t payload[8];
    payload[0]=(uint8_t)id; payload[1]=(uint8_t)(id>>8);
    payload[2]=(uint8_t)result; payload[3]=(uint8_t)((uint16_t)result>>8);
    putfloat(payload+4,value);
    (void)send_packet(4U,payload,sizeof(payload));
}
static void information(void)
{
    uint8_t payload[20];
    uint32_t key, token;
    key = Motor_Crit_Enter(); token = s_lease.token; Motor_Crit_Exit(key);
    put32(payload,STUDIO_PERIOD_US/1000U); put32(payload+4,STUDIO_TIMEOUT_US/1000U);
    put32(payload+8,token); put32(payload+12,STUDIO_PARAM_COUNT); put32(payload+16,MOTOR_SELECTED_MODEL);
    (void)send_packet(2U,payload,sizeof(payload));
}
bool Cmd_StudioIsActive(void) { return s_active; }
bool Cmd_StudioOwnsControl(void) { return s_lease.active; }

/** @description: 校验目标并写单槽邮箱，损坏/过期/重放帧不延长租约；仅 Cmd_Task。 */
void Cmd_StudioTargetLine(const char *line)
{
    studio_target_t target;
    uint32_t now = Hal_Motor_TimeUs(), key;
    if (!s_active || !Studio_ParseTarget(line,&target)) return;
    if (fabsf(target.rpm)>motor.ol_max_rpm || target.iq*target.iq+target.id*target.id >
        motor.speed_iq_limit*motor.speed_iq_limit || Motor_Position_IsEnabled() || Motor_Joint_IsEnabled() ||
        Motor_Bldc_IsSelected() || Motor_Detect_IsActive() || Motor_Hall_DetectIsActive() ||
        Motor_Protect_IsFaultLatched() || motor.ol_mode != MOTOR_OL_MODE_IF) return;
    key = Motor_Crit_Enter();
    if (Studio_LeaseAccept(&s_lease,&target,now)) { s_pending_target=target; s_pending=true; }
    else if (s_lease.expired) s_stop_requested=true;
    Motor_Crit_Exit(key);
}

/** @description: Plot 后台任务轮询，检查超时/消费目标/每 20 ms 采样发送。 */
void Cmd_StudioPoll(void)
{
    uint32_t now = Hal_Motor_TimeUs(), key, flags;
    studio_target_t target;
    bool take, stop;
    motor_command_t command;
    motor_status_t status;
    uint8_t payload[72];
    int32_t scheduler_lock;
    if (!s_active) return;
    /* 仅锁任务调度，不屏蔽 FOC 中断。使目标应用与 Shell 停机/释放不会交错重启。 */
    scheduler_lock=osKernelLock();
    if (scheduler_lock<0) return;
    key=Motor_Crit_Enter();
    stop=Studio_LeaseExpire(&s_lease,now) || s_stop_requested;
    s_stop_requested=false;
    if (stop) s_pending=false;
    take=s_pending && s_lease.active;
    target=s_pending_target; s_pending=false;
    Motor_Crit_Exit(key);
    if (stop) { (void)Motor_RequestStop(&motor); s_result=-6; }
    if (take && Motor_GetCommand(&motor,&command)==MOTOR_OK) {
        float reference = fabsf(motor.ol_current_rpm)>1.0f ? motor.ol_current_rpm : command.target_rpm;
        if (command.mode != MOTOR_OL_MODE_IF || Motor_Position_IsEnabled() || Motor_Joint_IsEnabled() ||
            Motor_Bldc_IsSelected() || Motor_Detect_IsActive() || Motor_Hall_DetectIsActive() ||
            Motor_Protect_IsFaultLatched() || (command.enable && target.rpm*reference<0.0f)) {
            key=Motor_Crit_Enter(); s_lease.active=false; Motor_Crit_Exit(key);
            (void)Motor_RequestStop(&motor); s_result=-4;
        } else {
            command.target_rpm=target.rpm; command.iq_set=target.iq; command.id_set=target.id;
            s_result=Motor_SetCommand(&motor,&command);
            if (s_result==MOTOR_OK) s_applied_sequence=target.sequence;
            else { s_lease.active=false; (void)Motor_RequestStop(&motor); }
        }
    }
    (void)osKernelRestoreLock(scheduler_lock);
    if ((uint32_t)(now-s_publish_us)>=STUDIO_PERIOD_US) {
        s_publish_us=now;
        if (Motor_GetStatus(&motor,&status)==MOTOR_OK) {
            key=Motor_Crit_Enter();
            flags=(status.enable?1U:0U)|(status.speed_enable?2U:0U)|(status.fault_latched?4U:0U)|
                (status.feedback.speed_valid?8U:0U)|(s_lease.active?16U:0U)|(s_lease.expired?32U:0U)|
                ((uint32_t)status.mode<<8)|((uint32_t)status.feedback.used_source<<16);
            put32(payload+4,s_lease.token);
            Motor_Crit_Exit(key);
            put32(payload,now); put32(payload+8,s_applied_sequence); put32(payload+12,flags);
            put32(payload+16,(uint32_t)status.fault); put32(payload+20,(uint32_t)s_result);
            putfloat(payload+24,status.target_rpm); putfloat(payload+28,status.feedback.mech_rpm);
            putfloat(payload+32,status.current_rpm); putfloat(payload+36,status.ia);
            putfloat(payload+40,status.ib); putfloat(payload+44,status.ic);
            putfloat(payload+48,status.id); putfloat(payload+52,status.iq);
            putfloat(payload+56,status.v_bus); putfloat(payload+60,status.theta);
            putfloat(payload+64,status.id_target); putfloat(payload+68,status.iq_target);
            (void)send_packet(1U,payload,sizeof(payload));
        }
    }
    if (s_param_cursor<STUDIO_PARAM_COUNT) {
        uint16_t cursor=s_param_cursor;
        payload[0]=(uint8_t)cursor; payload[1]=0;
        payload[2]=STUDIO_PARAM_COUNT; payload[3]=0;
        putfloat(payload+4,field_value(cursor));
        if (send_packet(3U,payload,8U)) s_param_cursor=cursor+1U;
    }
}

static int set_field(unsigned index, const char *text)
{
    motor_settings_t settings;
    const studio_field_t *field;
    char *end;
    float value=strtof(text,&end);
    uint8_t *p;
    if (index>=STUDIO_CONFIG_COUNT || end==text || *end!='\0' || !isfinite(value)) return MOTOR_ERR_INVALID;
    field=&s_fields[index];
    if (!field->writable || motor.ol_enable || Cmd_StudioOwnsControl() || Motor_Detect_IsActive() ||
        Motor_Hall_DetectIsActive() || Motor_Bldc_IsSelected() || Motor_Position_IsEnabled() ||
        Motor_Joint_IsEnabled()) return MOTOR_ERR_BUSY;
    if (!Motor_Settings_Capture(&motor,&settings)) return MOTOR_ERR_INVALID;
    p=(uint8_t *)&settings.config+field->config_offset;
    if (field->type==1U) { if (value!=0.0f && value!=1.0f) return MOTOR_ERR_INVALID; *(bool *)p=value!=0.0f; }
    else if (field->type==2U) { if (value<0.0f || value>65535.0f || floorf(value)!=value) return MOTOR_ERR_INVALID; *(uint16_t *)p=(uint16_t)value; }
    else if (field->type==3U) { if (value<0.0f || value>32.0f || floorf(value)!=value) return MOTOR_ERR_INVALID; *(int *)p=(int)value; }
    else memcpy(p,&value,sizeof(value));
    return Motor_SetConfig(&motor,&settings.config);
}

static int studio_command(int argc, char *argv[])
{
    uint32_t key;
    if (argc<2 || strcmp(argv[1],"status")==0) { information(); return 0; }
    if (strcmp(argv[1],"on")==0 && argc==2) {
        if (!Cmd_SerialPlotEnsureTask()) { acknowledge(0xffffU,MOTOR_ERR_BUSY,0); return -1; }
        s_active=true; information(); return 0;
    }
    if (strcmp(argv[1],"release")==0 || strcmp(argv[1],"off")==0) {
        bool owned;
        key=Motor_Crit_Enter(); owned=s_lease.active; s_lease.active=false; s_pending=false; Motor_Crit_Exit(key);
        if (owned || strcmp(argv[1],"release")==0) (void)Motor_RequestStop(&motor);
        acknowledge(0xffffU,0,0);
        if (strcmp(argv[1],"off")==0) s_active=false;
        return 0;
    }
    if (!s_active) { acknowledge(0xffffU,MOTOR_ERR_BUSY,0); return -1; }
    if (strcmp(argv[1],"control")==0 && (argc==2 || argc==3)) {
        char *end;
        unsigned long token = argc==3 ? strtoul(argv[2],&end,10) : Hal_Motor_TimeUs();
        if ((argc==3 && (argv[2][0]<'0' || argv[2][0]>'9' || end==argv[2] || *end!='\0' ||
            strlen(argv[2])>10U || (strlen(argv[2])==10U && strcmp(argv[2],"4294967295")>0))) || token==0U) {
            acknowledge(0xffffU,MOTOR_ERR_INVALID,0); return -1;
        }
        if (motor.ol_enable || motor.ol_mode!=MOTOR_OL_MODE_IF || Motor_Position_IsEnabled() ||
            Motor_Joint_IsEnabled() || Motor_Bldc_IsSelected() || Motor_Detect_IsActive() ||
            Motor_Hall_DetectIsActive() || Motor_Protect_IsFaultLatched()) {
            acknowledge(0xffffU,MOTOR_ERR_BUSY,0); return -1;
        }
        key=Motor_Crit_Enter(); s_lease.token=(uint32_t)token-1U; Studio_LeaseArm(&s_lease,Hal_Motor_TimeUs()); s_pending=false; s_stop_requested=false; s_applied_sequence=0; s_result=0; Motor_Crit_Exit(key);
        information(); return 0;
    }
    if (strcmp(argv[1],"start")==0 && argc==2) {
        motor_command_t command;
        bool ready;
        key=Motor_Crit_Enter();
        ready=s_lease.active && s_lease.have_sequence && s_applied_sequence==s_lease.last_sequence &&
            (uint32_t)(Hal_Motor_TimeUs()-s_lease.last_us)<STUDIO_TIMEOUT_US;
        Motor_Crit_Exit(key);
        if (!ready || Motor_Protect_IsFaultLatched() || Motor_Position_IsEnabled() || Motor_Joint_IsEnabled() ||
            Motor_Bldc_IsSelected() || Motor_Detect_IsActive() || Motor_Hall_DetectIsActive() ||
            Motor_GetCommand(&motor,&command)!=MOTOR_OK || command.mode!=MOTOR_OL_MODE_IF) {
            acknowledge(0xffffU,MOTOR_ERR_BUSY,0); return -1;
        }
        command.enable=true;
        acknowledge(0xffffU,Motor_SetCommand(&motor,&command),0);
        return 0;
    }
    if (strcmp(argv[1],"params")==0 && argc==2) { s_param_cursor=0; return 0; }
    if (strcmp(argv[1],"set")==0 && argc==4) {
        char *end;
        unsigned long id=strtoul(argv[2],&end,10);
        int result=(*end=='\0' && end!=argv[2] && id<STUDIO_CONFIG_COUNT) ? set_field((unsigned)id,argv[3]) : MOTOR_ERR_INVALID;
        acknowledge((uint16_t)id,result,id<STUDIO_PARAM_COUNT ? field_value((unsigned)id) : 0.0f);
        return result;
    }
    acknowledge(0xffffU,MOTOR_ERR_INVALID,0);
    return -1;
}

int Cmd_Studio(int argc, char *argv[])
{
    int32_t locked=osKernelLock();
    int result;
    if (locked<0) return MOTOR_ERR_BUSY;
    result=studio_command(argc,argv);
    (void)osKernelRestoreLock(locked);
    return result;
}
#endif
