/* 磁链扫描：拟合磁链/偏置、失败场景与参数只读恢复。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_param.h"
#include "motor_detect.h"
#include "flux_scan_constants.inc"
#define PRINTF(...) ((void)0)
#define MOTOR_DETECT_STAGE_FLUX 3
MOTOR_PARAM_S motor;
static motor_detect_result_t s_detect_result;
static bool s_detect_active,s_flux_scan,encoder_ok;
static unsigned calls, fail_stage;
static int scenario;
/**
 * @description: 主机测试替身： 打印滑行磁链诊断轨迹缓冲。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
static void Detect_PrintFluxTrace(void) {}
static struct {
 float electrical_rpm_target,measured_resistance,flux_duty_still,flux_duty_now,flux_duty_max;
 float electrical_rpm_now,resistance_voltage_avg;
 uint32_t samples,flux_encoder_count,flux_encoder_attempts;
 float flux_encoder_omega_sum,flux_encoder_voltage_sum;
 float flux_pair_time;
} s_detect_control;
/**
 * @description: 主机测试替身：查询编码器对照采集是否打开。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 测试记录值 encoder_ok
 */
static bool Motor_Encoder_IsCompareEnabled(void) { return encoder_ok; }
/**
 * @description: 主机测试替身：查询默认实例本拍转速是否有效
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 测试记录值 encoder_ok
 */
static bool Motor_Encoder_IsSpeedValid(void) { return encoder_ok; }
/**
 * @description: 主机测试替身： 将值钳位到 [minimum, maximum]。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} x x 参数
 * @param {float} lo lo 参数
 * @param {float} hi hi 参数
 * @return {float} 本次计算或查询的结果
 */
static float Detect_Clamp(float x,float lo,float hi) {return fminf(hi,fmaxf(lo,x));}
/**
 * @description: 主机测试替身： R/L 检测主流程：多点拟合电阻，再测 dq 电感。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} current 兼容参数，本实现不使用
 * @param {motor_detect_result_t *} out 兼容参数，本实现不使用
 * @param {bool} retain retain 参数
 * @return {bool} true=R/L 检测完成且结果有效；false=启动失败、中止或检测失败
 */
static bool Detect_MotorRlInternal(float current,motor_detect_result_t *out,bool retain) {
 (void)current;(void)out;assert(retain);memset(&s_detect_result,0,sizeof(s_detect_result));
 s_detect_result.resistance_valid=s_detect_result.inductance_valid=true;
 s_detect_result.resistance_ohm=3.75f;s_detect_result.inductance_h=.0013f;s_detect_active=true;return true;
}
/**
 * @description: 主机测试替身： 运行一个检测阶段直到完成或超时，负责启停 PWM。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {int} stage 检测阶段
 * @param {float} current 兼容参数，本实现不使用
 * @param {uint32_t} timeout 兼容参数，本实现不使用
 * @return {bool} 本次处理的判定结果，具体条件见职责与实现说明
 */
static bool Detect_RunStage(int stage,float current,uint32_t timeout) {
 (void)current;(void)timeout;assert(stage==3 && s_detect_active);calls++;
 if(calls==fail_stage){s_detect_result.error=MOTOR_DETECT_ERROR_USER_ABORT;return false;}
 float erpm=scenario==3 ? 2000 : s_detect_control.electrical_rpm_target;
 float omega=erpm*(2*MOTOR_DETECT_PI/60);
 float voltage=.0035f*omega+.1f;
 if(scenario==4 && calls==2)voltage+=1;
 s_detect_control.electrical_rpm_now=erpm;
 s_detect_control.resistance_voltage_avg=voltage;
 s_detect_control.flux_encoder_attempts=64;s_detect_control.flux_encoder_count=scenario==1 ? 10 : 64;
 s_detect_control.flux_pair_time=.1f;
 s_detect_control.flux_encoder_omega_sum=omega*s_detect_control.flux_pair_time*(scenario==2 ? .5f : 1);
 s_detect_control.flux_encoder_voltage_sum=voltage*s_detect_control.flux_pair_time;
 s_detect_result.flux_valid=true;s_detect_result.flux_linkage_wb=voltage/omega;return true;
}
#include "flux_scan_under_test.inc"
/**
 * @description: 初始化测试场景。
 * @return {void} 无
 */
static void init(void) {
 memset(&motor,0,sizeof(motor));motor.ol_current_kp=.12f;motor.ol_current_ki=18;motor.ol_vq_limit=4;
 s_detect_active=s_flux_scan=false;encoder_ok=true;calls=fail_stage=0;scenario=0;
}
/**
 * @description: 检查测试中的状态恢复结果。
 * @return {void} 无
 */
static void restored(void) {assert(!s_detect_active && !s_flux_scan);assert(motor.ol_current_kp==.12f && motor.ol_current_ki==18 && motor.ol_vq_limit==4);}
/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void) {
 motor_detect_result_t out;
 init();assert(DETECT_MotorFluxScan(.5f,3000,.3f,&out));assert(calls==3 && out.flux_scan_valid);
 assert(fabsf(out.flux_scan_fit_wb-.0035f)<1e-7f && fabsf(out.flux_scan_offset_v-.1f)<1e-5f);restored();
 for(int i=1;i<=4;i++){init();scenario=i;assert(!DETECT_MotorFluxScan(.5f,3000,.3f,&out));assert(out.error==MOTOR_DETECT_ERROR_FLUX);restored();}
 init();encoder_ok=false;assert(!DETECT_MotorFluxScan(.5f,3000,.3f,&out) && calls==0);
 init();assert(!DETECT_MotorFluxScan(.5f,4200,.3f,&out) && calls==0);
 init();assert(!DETECT_MotorFluxScan(.5f,NAN,.3f,&out) && calls==0);
 init();fail_stage=2;assert(!DETECT_MotorFluxScan(.5f,3000,.3f,&out));assert(calls==2 && out.error==MOTOR_DETECT_ERROR_USER_ABORT);restored();
 init();encoder_ok=false;assert(DETECT_MotorParameters(.5f,3000,.3f,&out));assert(calls==1 && !out.flux_scan_valid);restored();
 puts("motor flux scan: pass");return 0;
}
