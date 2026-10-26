/* 无感速度命令(sltest)主机测试：流程、跟踪校验与故障报告 */
/* Includes ------------------------------------------------------------------*/
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
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 替代固件打印并记录测试输出。
 * @param {const char *} format format 参数
 * @param {...} ... 与格式字符串匹配的可变参数
 * @return {void} 无
 */
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
/**
 * @description: 主机测试替身：读取 PWM 周期计数（通常等于 ARR）
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {uint32_t} 测试记录值 5600
 */
uint32_t Hal_Motor_PwmPeriodCounts(void) { return 5600; }
/**
 * @description: 主机测试替身：读取电机通用微秒时基（自由运行，非 Hall 专用）
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {uint32_t} 测试记录值 clock_us
 */
uint32_t Hal_Motor_TimeUs(void) { return clock_us; }
/**
 * @description: 主机测试替身：模拟任务延时，按本测试场景推进时间或注入事件。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {uint32_t} ms ms 参数
 * @return {void} 无
 */
static void osDelay(uint32_t ms) { clock_us += ms * 1000U; }
/**
 * @description: 主机测试替身：测试过程中是否收到任意按键。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 测试记录值 abort_us && clock_us >= abort_us
 */
static bool Cmd_Speed_StopKey(void) { return abort_us && clock_us >= abort_us; }
/**
 * @description: 主机测试替身：捕获当前配置快照，供命令修改后整体提交。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {motor_config_t *} out 输出缓冲或结果
 * @return {bool} 本实现固定返回 true
 */
static bool Cmd_Ol_CaptureConfig(motor_config_t *out) { *out=config; return true; }
/**
 * @description: 主机测试替身：从电机对象捕获当前命令字段。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {const MOTOR_PARAM_S *} m 兼容参数，本实现不使用
 * @param {motor_command_t *} out 输出缓冲或结果
 * @return {motor_result_t} 测试记录值 MOTOR_OK
 */
motor_result_t Motor_GetCommand(const MOTOR_PARAM_S *m, motor_command_t *out) { (void)m; *out=command; return MOTOR_OK; }
/**
 * @description: 主机测试替身：校验并整体提交命令到电机对象。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {MOTOR_PARAM_S *} m 兼容参数，本实现不使用
 * @param {const motor_command_t *} c c 参数
 * @return {motor_result_t} 测试记录值 MOTOR_OK
 */
motor_result_t Motor_SetCommand(MOTOR_PARAM_S *m, const motor_command_t *c)
{
    (void)m;
    if (c->enable && c->target_rpm > 800)
        assert(config.sensorless_takeover && c->speed_enable);
    if (c->enable)
        assert(fabsf(config.speed_iq_limit - (motor.v_bus > 22.0f ? 1.0f :
                    (c->target_rpm > 1800 ? .30f : .15f))) < 1e-6f);
    command = *c;
    motor.ol_enable = c->enable;
    return MOTOR_OK;
}
/**
 * @description: 主机测试替身：获取一致的只读运行快照。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {const MOTOR_PARAM_S *} m 兼容参数，本实现不使用
 * @param {motor_status_t *} s s 参数
 * @return {motor_result_t} MOTOR_OK 或 MOTOR_ERR_NULL
 */
motor_result_t Motor_GetStatus(const MOTOR_PARAM_S *m,motor_status_t *s) { (void)m; memset(s,0,sizeof(*s));s->enable=command.enable;s->target_rpm=command.target_rpm;s->iq=current_failure && clock_us>=1000000U ? 1.25f : .3f;return read_failure && clock_us>=1000000U ? (motor_result_t)-1 : MOTOR_OK; }
/**
 * @description: 主机测试替身：校验并应用配置（不改校准与运行命令目标）。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {MOTOR_PARAM_S *} m 兼容参数，本实现不使用
 * @param {const motor_config_t *} c c 参数
 * @return {motor_result_t} 测试记录值 MOTOR_OK
 */
motor_result_t Motor_SetConfig(MOTOR_PARAM_S *m,const motor_config_t *c) { (void)m;config=*c;command.speed_enable=c->speed_enable;return MOTOR_OK; }
/**
 * @description: 主机测试替身：提交停机命令并清除斜坡/电流积分（不清故障锁存）。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {MOTOR_PARAM_S *} m 兼容参数，本实现不使用
 * @return {motor_result_t} 测试记录值 MOTOR_OK
 */
motor_result_t Motor_RequestStop(MOTOR_PARAM_S *m) { (void)m;command.enable=false;motor.ol_enable=false;stops++;return MOTOR_OK; }
/**
 * @description: 主机测试替身：读取当前请求的反馈来源。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {motor_feedback_source_t} 测试记录值 source
 */
motor_feedback_source_t Motor_GetFeedbackSource(void) { return source; }
/**
 * @description: 主机测试替身：设置转子反馈来源（开环/无感/编码器/霍尔）。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {motor_feedback_source_t} s s 参数
 * @return {motor_result_t} 测试记录值 MOTOR_OK
 */
motor_result_t Motor_SetFeedbackSource(motor_feedback_source_t s) { source=s;return MOTOR_OK; }
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
 * @description: 主机测试替身：查询默认实例是否已锁存故障。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 测试记录值 fault
 */
bool Motor_Protect_IsFaultLatched(void) { return fault; }
/**
 * @description: 主机测试替身：开关位置环。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {bool} x 兼容参数，本实现不使用
 * @return {bool} 本实现固定返回 true
 */
static bool Motor_Position_Enable(bool x) { (void)x;return true; }
/**
 * @description: 主机测试替身：开关关节阻抗。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {bool} x 兼容参数，本实现不使用
 * @return {bool} 本实现固定返回 true
 */
static bool Motor_Joint_Enable(bool x) { (void)x;return true; }
static int trace_mode;
/**
 * @description: 主机测试替身：复位环形诊断索引并开始记录。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
void Motor_Timing_TraceArm(void) { trace_mode=0; }
/**
 * @description: 主机测试替身：按接管触发方式武装环形诊断。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
void Motor_Timing_TraceArmEntry(void) { trace_mode=1; }
/**
 * @description: 主机测试替身：首次 CLOSED 后逐控制拍记录 384 条。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
void Motor_Timing_TraceArmEntryFast(void) { trace_mode=2; }
/**
 * @description: 主机测试替身：按接管触发方式武装轻量轨迹。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
void Motor_Timing_TraceArmLight(void) { trace_mode=3; }
/**
 * @description: 主机测试替身：武装飞行记录，复用轻量缓冲。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
void Motor_Timing_TraceArmFlight(void) { trace_mode=4; }
/**
 * @description: 主机测试替身：停止继续写入环形诊断。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
void Motor_Timing_TraceStop(void) {}
/**
 * @description: 主机测试替身：停止节拍和 DMA 回调累加。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
void Motor_Timing_CadenceStop(void) {}
/**
 * @description: 主机测试替身：获取默认实例观测器状态机当前状态。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {motor_sensorless_state_t} OPEN_LOOP、LOCKING 或 CLOSED
 */
motor_sensorless_state_t Motor_Sensorless_GetState(void) { return config.sensorless_takeover && !never_lock && !(lose_lock && clock_us>12000000U) ? MOTOR_SENSORLESS_CLOSED : MOTOR_SENSORLESS_LOCKING; }
/**
 * @description: 主机测试替身：获取默认实例低通后的机械转速。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {float} 机械转速 (RPM)，反转时为负
 */
float Motor_Sensorless_GetMechanicalRpm(void) { return underspeed && command.target_rpm>=2200 ? .87f*command.target_rpm : command.target_rpm; }
/**
 * @description: 主机测试替身：查询默认实例速度反馈是否可信
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} true=反馈可信
 */
bool Motor_SpeedLoop_IsFeedbackValid(void) { return command.speed_enable && Motor_Sensorless_GetState()==MOTOR_SENSORLESS_CLOSED; }
/**
 * @description: 主机测试替身：查询编码器对照采集是否打开。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 本实现固定返回 false
 */
bool Motor_Encoder_IsCompareEnabled(void) { return false; }
/**
 * @description: 主机测试替身：查询默认实例本拍转速是否有效
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 本实现固定返回 false
 */
bool Motor_Encoder_IsSpeedValid(void) { return false; }
/**
 * @description: 主机测试替身：读取默认实例距最近一次 SSI/I2C 新帧的时间。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {float} 测试记录值 1
 */
float Motor_Encoder_GetSampleAge(void) { return 1; }
/**
 * @description: 主机测试替身：读取默认实例已更新的机械转速
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {float} 本实现固定返回 0
 */
float Motor_Encoder_GetMechRpm(void) { return 0; }
#include "sltest_under_test.inc"
/**
 * @description: 初始化测试场景。
 * @return {void} 无
 */
static void init(void) {
 memset(&motor,0,sizeof(motor));memset(&command,0,sizeof(command));memset(&config,0,sizeof(config));
 config.ol_max_rpm=1000;config.sensorless_transition_rpm=500;config.sensorless_fallback_rpm=300;
 config.ol_vq_limit=4;
 config.sensorless_motor_r=2.925f;config.sensorless_motor_l=.00086f;config.sensorless_flux_linkage=.0035f;
 motor.v_bus=16;motor.ol_pole_pairs=7;source=MOTOR_FEEDBACK_SRC_OPENLOOP;
 clock_us=abort_us=stops=output_size=0;output[0]=0;current_failure=read_failure=false;fault=never_lock=lose_lock=false;
 underspeed=false;
}
/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
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
 {
  char *off[]={"sltest","--trace=off","2","600"};
  char *entry[]={"sltest","--trace=entry","2","600"};
  char *entryfast[]={"sltest","--trace=entryfast","2","600"};
  char *invalid[]={"sltest","--trace=bad","2","600"};
  init();saved=config;assert(Cmd_SensorlessTest(4,off)==0 && trace_mode==0 && !command.enable);
  assert(memcmp(&saved,&config,sizeof(saved))==0);
  init();assert(Cmd_SensorlessTest(4,entry)==0 && trace_mode==1 && !command.enable);
  init();assert(Cmd_SensorlessTest(4,entryfast)==0 && trace_mode==2 && !command.enable);
  init();char *lite[]={"sltest","--trace=lite","2","600"};
  assert(Cmd_SensorlessTest(4,lite)==0 && trace_mode==3 && !command.enable);
  init();char *flight[]={"sltest","--trace=flight","2","600"};
  assert(Cmd_SensorlessTest(4,flight)==0 && trace_mode==4 && !command.enable);
  init();assert(Cmd_SensorlessTest(1,args)==0 && trace_mode==3 && !command.enable);
  init();assert(Cmd_SensorlessTest(4,invalid)==-1 && stops==0);
  {
   char *m3508[]={"sltest","--3508","--trace=off","2","600"};
   char *m3508high[]={"sltest","--3508","2","800"};
   init();motor.v_bus=24.0f;saved=config;
   assert(Cmd_SensorlessTest(5,m3508)==0 && !command.enable && trace_mode==0);
   assert(memcmp(&saved,&config,sizeof(saved))==0);
   init();motor.v_bus=16.0f;assert(Cmd_SensorlessTest(5,m3508)==-1 && stops==0);
   init();motor.v_bus=24.0f;assert(Cmd_SensorlessTest(4,m3508high)==-1 && stops==0);
   init();motor.v_bus=24.0f;assert(Cmd_SensorlessTest(1,args)==-1 && stops==0);
  }
 }
 puts("sensorless speed command: pass");return 0;
}
