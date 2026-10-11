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
static void Detect_PrintFluxTrace(void) {}
static struct {
 float electrical_rpm_target,measured_resistance,flux_duty_still,flux_duty_now,flux_duty_max;
 float electrical_rpm_now,resistance_voltage_avg;
 uint32_t samples,flux_encoder_count,flux_encoder_attempts;
 float flux_encoder_omega_sum,flux_encoder_voltage_sum;
 float flux_pair_time;
} s_detect_control;
static bool Motor_Encoder_IsCompareEnabled(void) { return encoder_ok; }
static bool Motor_Encoder_IsSpeedValid(void) { return encoder_ok; }
static float Detect_Clamp(float x,float lo,float hi) {return fminf(hi,fmaxf(lo,x));}
static bool Detect_MotorRlInternal(float current,motor_detect_result_t *out,bool retain) {
 (void)current;(void)out;assert(retain);memset(&s_detect_result,0,sizeof(s_detect_result));
 s_detect_result.resistance_valid=s_detect_result.inductance_valid=true;
 s_detect_result.resistance_ohm=3.75f;s_detect_result.inductance_h=.0013f;s_detect_active=true;return true;
}
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
static void init(void) {
 memset(&motor,0,sizeof(motor));motor.ol_current_kp=.12f;motor.ol_current_ki=18;motor.ol_vq_limit=4;
 s_detect_active=s_flux_scan=false;encoder_ok=true;calls=fail_stage=0;scenario=0;
}
static void restored(void) {assert(!s_detect_active && !s_flux_scan);assert(motor.ol_current_kp==.12f && motor.ol_current_ki==18 && motor.ol_vq_limit==4);}
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
