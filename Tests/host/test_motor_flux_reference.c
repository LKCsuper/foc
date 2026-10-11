#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#define MOTOR_DETECT_PI 3.14159265358979323846f
#define MOTOR_DETECT_FLUX_MIN_BEMF_V .08f
static bool s_flux_scan=true,s_flux_raw_valid;
static float s_flux_raw_angle,angle,age,s_flux_bemf_unwrapped,s_flux_raw_bemf;
static bool valid=true,fault; static uint32_t s_flux_raw_stamp,stamp;
static float rpm;
static struct {unsigned samples,flux_encoder_attempts,flux_encoder_count;float flux_coast_elapsed,flux_encoder_omega_sum,flux_encoder_voltage_sum,flux_pair_bemf_angle,flux_pair_time;float electrical_rpm_target,flux_duty_max,flux_duty_now;} s_detect_control;
typedef struct {float ol_pole_pairs;} Params;
static float Motor_Encoder_GetSampleAge(void){return age;}
static uint32_t Motor_Encoder_GetSampleTimeUs(void){return stamp;}
static float Motor_Encoder_GetMechAngle(void){return angle;}
static bool Motor_Encoder_IsSpeedValid(void){return valid;}
static bool Motor_Encoder_IsSignalFault(void){return fault;}
static float Motor_Encoder_GetMechRpm(void){return rpm;}
static bool rotation_invalid(void){Params m={7};Params *motor_param=&m;
#include "flux_rotation_under_test.inc"
return rotation_invalid || s_detect_control.flux_duty_now < minimum_run_duty;
}
static void step(float t,float a){Params m={7};Params *motor_param=&m;float voltage_mag=.4f;angle=a;stamp=(uint32_t)((t-age)*1e6f);s_flux_bemf_unwrapped=t*70;s_detect_control.samples+=16;s_detect_control.flux_coast_elapsed=t;
#include "flux_reference_under_test.inc"
}
static void reset(void){memset(&s_detect_control,0,sizeof(s_detect_control));s_flux_raw_valid=false;s_flux_raw_angle=age=s_flux_bemf_unwrapped=s_flux_raw_bemf=0;s_flux_raw_stamp=stamp=0;valid=true;fault=false;}
int main(void){
 reset();step(.02f,6.28f);step(.021f,.0068146928f);assert(s_detect_control.flux_encoder_count==1);assert(fabsf(s_detect_control.flux_encoder_omega_sum-.07f)<.0001f);
 assert(fabsf(s_detect_control.flux_pair_bemf_angle-.07f)<.0001f);
 assert(fabsf(s_detect_control.flux_pair_time-.001f)<1e-6f);
 step(.023f,.0268146928f);
 assert(fabsf(s_detect_control.flux_encoder_omega_sum/s_detect_control.flux_pair_time-70)<.02f);
 assert(fabsf(s_detect_control.flux_pair_bemf_angle/s_detect_control.flux_pair_time-70)<.02f);
 assert(fabsf(s_detect_control.flux_encoder_voltage_sum/s_detect_control.flux_pair_time-.4f)<1e-5f);
 reset();step(.02f,.006f);step(.021f,6.279185307f);assert(s_detect_control.flux_encoder_count==0);
 reset();step(.02f,1);step(.03f,1.1f);assert(s_detect_control.flux_encoder_count==0);
 reset();step(.02f,1);age=.006f;step(.021f,1.01f);assert(!s_flux_raw_valid && s_detect_control.flux_encoder_count==0);
 reset();step(.02f,1);fault=true;step(.021f,1.01f);assert(!s_flux_raw_valid && s_detect_control.flux_encoder_count==0);
 reset();step(.02f,1);step(.021f,1);assert(s_detect_control.flux_encoder_count==0);
 reset();s_detect_control.electrical_rpm_target=1800;s_detect_control.flux_duty_now=.075f;s_detect_control.flux_duty_max=.11f;rpm=1800/7.0f;
 assert(!rotation_invalid());rpm=1000/7.0f;assert(rotation_invalid());rpm=1800/7.0f;age=.006f;assert(rotation_invalid());age=0;fault=true;assert(rotation_invalid());fault=false;s_detect_control.flux_duty_now=.02f;assert(rotation_invalid());
 s_flux_scan=false;s_detect_control.flux_duty_now=.075f;assert(rotation_invalid());s_detect_control.flux_duty_now=.09f;assert(!rotation_invalid());
 puts("flux raw reference: pass");return 0;
}
