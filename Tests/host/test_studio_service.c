/* Studio 服务命令主机测试：显式启动、重放、TX 预算与配置守卫 */
/* Includes ------------------------------------------------------------------*/
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
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 主机测试替身：进入短临界区（屏蔽 IRQ），返回进入前的 PRIMASK。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {uint32_t} 本实现固定返回 0
 */
uint32_t Motor_Crit_Enter(void) { return 0; }
/**
 * @description: 主机测试替身：退出短临界区；仅当进入前 IRQ 开启时恢复。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {uint32_t} key 兼容参数，本实现不使用
 * @return {void} 无
 */
void Motor_Crit_Exit(uint32_t key) { (void)key; }
/**
 * @description: 主机测试替身：读取电机通用微秒时基（自由运行，非 Hall 专用）
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {uint32_t} 测试记录值 test_now
 */
uint32_t Hal_Motor_TimeUs(void) { return test_now; }
/**
 * @description: 主机测试替身：模拟调度器锁定并返回测试指定的先前状态。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {int32_t} 本实现固定返回 0
 */
int32_t osKernelLock(void) { return 0; }
/**
 * @description: 主机测试替身：模拟恢复先前调度器锁状态。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {int32_t} lock lock 参数
 * @return {int32_t} 测试记录值 lock
 */
int32_t osKernelRestoreLock(int32_t lock) { return lock; }
/**
 * @description: 主机测试替身：查询位置环是否使能。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 测试记录值 other_control
 */
bool Motor_Position_IsEnabled(void) { return other_control; }
/**
 * @description: 主机测试替身：查询关节阻抗是否使能。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 本实现固定返回 false
 */
bool Motor_Joint_IsEnabled(void) { return false; }
/**
 * @description: 主机测试替身：查询 BLDC 是否占用主 PWM 出口。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 本实现固定返回 false
 */
bool Motor_Bldc_IsSelected(void) { return false; }
/**
 * @description: 主机测试替身：偏移校准时 FOC 不得改占空比编排（ISR 仍采样+软件保护）。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 本实现固定返回 false
 */
bool Motor_Detect_IsActive(void) { return false; }
/**
 * @description: 主机测试替身：霍尔标定是否占用 FOC 中断路径
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 本实现固定返回 false
 */
bool Motor_Hall_DetectIsActive(void) { return false; }
/**
 * @description: 主机测试替身：查询默认实例是否已锁存故障。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 本实现固定返回 false
 */
bool Motor_Protect_IsFaultLatched(void) { return false; }
/**
 * @description: 主机测试替身：确保串口绘图后台任务存在，供 studio 复用其采样节拍。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 本实现固定返回 true
 */
bool Cmd_SerialPlotEnsureTask(void) { return true; }
/**
 * @description: 主机测试替身：把一整块字节写入 UART DMA 环形队列。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {const uint8_t *} data 数据缓冲
 * @param {ULONG} n n 参数
 * @return {bool} true=整帧已入队；false=参数非法或剩余队列空间不足，不等待 DMA
 */
bool Bsp_Usart_TryWriteFrame(const uint8_t *data, ULONG n) { if(transport_full)return false;memcpy(last_packet,data,n);writes++;return true; }
/**
 * @description: 主机测试替身：从电机对象捕获当前命令字段。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {const MOTOR_PARAM_S *} m 电机实例
 * @param {motor_command_t *} c c 参数
 * @return {motor_result_t} 测试记录值 MOTOR_OK
 */
motor_result_t Motor_GetCommand(const MOTOR_PARAM_S *m,motor_command_t *c) { memset(c,0,sizeof(*c));c->enable=m->ol_enable;c->mode=m->ol_mode;c->target_rpm=m->ol_target_rpm;c->iq_set=m->ol_iq_set;c->id_set=m->ol_id_set;return MOTOR_OK; }
/**
 * @description: 主机测试替身：校验并整体提交命令到电机对象。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {MOTOR_PARAM_S *} m 电机实例
 * @param {const motor_command_t *} c c 参数
 * @return {motor_result_t} 测试记录值 MOTOR_OK
 */
motor_result_t Motor_SetCommand(MOTOR_PARAM_S *m,const motor_command_t *c) { if(c->enable&&!m->ol_enable)starts++;m->ol_enable=c->enable;m->ol_target_rpm=c->target_rpm;m->ol_iq_set=c->iq_set;m->ol_id_set=c->id_set;return MOTOR_OK; }
/**
 * @description: 主机测试替身：提交停机命令并清除斜坡/电流积分（不清故障锁存）。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {MOTOR_PARAM_S *} m 电机实例
 * @return {motor_result_t} 测试记录值 MOTOR_OK
 */
motor_result_t Motor_RequestStop(MOTOR_PARAM_S *m) { m->ol_enable=false;stops++;return MOTOR_OK; }
/**
 * @description: 主机测试替身：获取一致的只读运行快照。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {const MOTOR_PARAM_S *} m 电机实例
 * @param {motor_status_t *} s s 参数
 * @return {motor_result_t} 测试记录值 MOTOR_OK
 */
motor_result_t Motor_GetStatus(const MOTOR_PARAM_S *m,motor_status_t *s) { memset(s,0,sizeof(*s));s->enable=m->ol_enable;s->mode=m->ol_mode;s->target_rpm=m->ol_target_rpm;return MOTOR_OK; }
/**
 * @description: 主机测试替身：从旧电机对象提取配置/校准；调用者保证快照一致，不提取命令和积分。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {const MOTOR_PARAM_S *} m 电机实例
 * @param {motor_settings_t *} s s 参数
 * @return {bool} 本实现固定返回 true
 */
bool Motor_Settings_Capture(const MOTOR_PARAM_S *m,motor_settings_t *s) { memset(s,0,sizeof(*s));s->config.speed_iq_limit=m->speed_iq_limit;return true; }
/**
 * @description: 主机测试替身：校验并应用配置（不改校准与运行命令目标）。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {MOTOR_PARAM_S *} m 电机实例
 * @param {const motor_config_t *} c c 参数
 * @return {motor_result_t} 测试记录值 MOTOR_OK
 */
motor_result_t Motor_SetConfig(MOTOR_PARAM_S *m,const motor_config_t *c) { config_sets++;m->speed_kp=c->speed_kp;return MOTOR_OK; }
/* 由测试入口从真实 cmd_studio.c 提取，不复制实现。 */
#include "studio_service_under_test.inc"
/**
 * @description: 构造测试用命令。
 * @param {const char *} sub sub 参数
 * @param {const char *} a a 参数
 * @param {const char *} b b 参数
 * @return {int} 本次计算或查询的结果
 */
static int command(const char *sub,const char *a,const char *b)
{
    char *argv[4]={(char *)"studio",(char *)sub,(char *)a,(char *)b};
    return Cmd_Studio(b?4:a?3:2,argv);
}
/**
 * @description: 生成或检查本次测试目标。
 * @param {unsigned} sequence sequence 参数
 * @param {const char *} rpm 机械转速 (RPM)
 * @param {const char *} iq Q 轴电流 (A)
 * @return {void} 无
 */
static void target(unsigned sequence,const char *rpm,const char *iq)
{
    char body[128],line[160];
    sprintf(body,"@C1,%lu,%u,%s,%s,0",(unsigned long)s_lease.token,sequence,rpm,iq);
    sprintf(line,"%s*%04X",body,Studio_Crc16((const uint8_t *)body,strlen(body)));
    Cmd_StudioTargetLine(line);
}
/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
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
