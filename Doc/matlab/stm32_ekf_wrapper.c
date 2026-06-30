
/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#define SIMPLIFIED_RTWTYPES_COMPATIBILITY
#include "rtwtypes.h"
#undef SIMPLIFIED_RTWTYPES_COMPATIBILITY
#endif


#define SIMULINK_USE_ARM_MATH
/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
//#define SIMULINK_USE_ARM_MATH
#ifdef  SIMULINK_USE_ARM_MATH  
#include "arm_math.h"
#else
#include <math.h>
#endif

float Rs;
float Ls;
float flux;

float Q_0_0;
float Q_0_1;
float Q_0_2;
float Q_0_3;
float Q_1_0;
float Q_1_1;
float Q_1_2;
float Q_1_3;
float Q_2_0;
float Q_2_1;
float Q_2_2;
float Q_2_3;
float Q_3_0;
float Q_3_1;
float Q_3_2;
float Q_3_3;

float R_0_0;
float R_0_1;
float R_1_0;
float R_1_1;
float T;

float vs_ab_0_0;
float vs_ab_1_0;
float is_ab_0_0;
float is_ab_1_0;

float P0_0_0;
float P0_0_1;
float P0_0_2;
float P0_0_3;
float P0_1_0;
float P0_1_1;
float P0_1_2;
float P0_1_3;
float P0_2_0;
float P0_2_1;
float P0_2_2;
float P0_2_3;
float P0_3_0;
float P0_3_1;
float P0_3_2;
float P0_3_3;

float H_0_0;
float H_0_1;
float H_0_2;
float H_0_3;
float H_1_0;
float H_1_1;
float H_1_2;
float H_1_3;

float B_0_0;
float B_0_1;
float B_1_0;
float B_1_1;
float B_2_0;
float B_2_1;
float B_3_0;
float B_3_1;

float F_0_0;
float F_0_1;
float F_0_2;
float F_0_3;
float F_1_0;
float F_1_1;
float F_1_2;
float F_1_3;
float F_2_0;
float F_2_1;
float F_2_2;
float F_2_3;
float F_3_0;
float F_3_1;
float F_3_2;
float F_3_3;

float temp_0_0;
float temp_0_1;
float temp_0_2;
float temp_0_3;
float temp_1_0;
float temp_1_1;
float temp_1_2;
float temp_1_3;
float temp_2_0;
float temp_2_1;
float temp_2_2;
float temp_2_3;
float temp_3_0;
float temp_3_1;
float temp_3_2;
float temp_3_3;

float f1_0_0;
float f1_1_0;
float f1_2_0;
float f1_3_0;

float f2_0_0;
float f2_0_1;
float f2_0_2;
float f2_0_3;
float f2_1_0;
float f2_1_1;
float f2_1_2;
float f2_1_3;
float f2_2_0;
float f2_2_1;
float f2_2_2;
float f2_2_3;
float f2_3_0;
float f2_3_1;
float f2_3_2;
float f2_3_3;

float X_pred_0_0;
float X_pred_1_0;
float X_pred_2_0;
float X_pred_3_0;

float Y_pred_0_0;
float Y_pred_1_0;

float Y_0_0;
float Y_1_0;

float P_pred_0_0;
float P_pred_0_1;
float P_pred_0_2;
float P_pred_0_3;    
float P_pred_1_0;
float P_pred_1_1;
float P_pred_1_2;
float P_pred_1_3;  
float P_pred_2_0;
float P_pred_2_1;
float P_pred_2_2;
float P_pred_2_3;
float P_pred_3_0;
float P_pred_3_1;
float P_pred_3_2;
float P_pred_3_3;

float temp_0_0_t;
float temp_0_1_t;
float temp_1_0_t;
float temp_1_1_t;
float temp;

float K_0_0;
float K_0_1;    
float K_1_0;
float K_1_1;   
float K_2_0;
float K_2_1;   
float K_3_0;
float K_3_1;

float tempa_0_0;
float tempa_1_0;
float tempa_2_0;
float tempa_3_0;
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 7
#define y_width 4

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
/* extern double func(double a); */
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Start function
 *
 */
void stm32_ekf_Start_wrapper(real_T *xD)
{
/* %%%-SFUNWIZ_wrapper_Start_Changes_BEGIN --- EDIT HERE TO _END */
/**********************************
             ??????
        ?????????QQ????B?
          QQ??1323279545
        ??????www.huiqudong.cn
**********************************/

Rs = 0.4f;//????
Ls = 0.00015f;//???
flux = 0.017643f;//????
    // rtU.Ls = 0.00015;                        // H
    // rtU.Rs = 0.4;                           // 欧姆
    // // 反电动势常数 3.2 v kprm
    // rtU.flux = 0.017643;                    /* 可以由反电动势常数转换 */

Q_0_0 = 0.1f;
Q_0_1 = 0.0f;
Q_0_2 = 0.0f;
Q_0_3 = 0.0f;
Q_1_0 = 0.0f;
Q_1_1 = 0.1f;
Q_1_2 = 0.0f;
Q_1_3 = 0.0f;
Q_2_0 = 0.0f;
Q_2_1 = 0.0f;
Q_2_2 = 0.1f;
Q_2_3 = 0.0f;
Q_3_0 = 0.0f;
Q_3_1 = 0.0f;
Q_3_2 = 0.0f;
Q_3_3 = 0.01f;

R_0_0 = 0.2f;
R_0_1 = 0.0f;
R_1_0 = 0.0f;
R_1_1 = 0.2f;
T = 0.0001f;

H_0_0 = 1.0f;
H_0_1 = 0.0f;
H_0_2 = 0.0f;
H_0_3 = 0.0f;
H_1_0 = 0.0f;
H_1_1 = 1.0f;
H_1_2 = 0.0f;
H_1_3 = 0.0f;

B_0_0 = 1.0f / 0.00015f;

B_0_1 = 0.0f;
B_1_0 = 0.0f;
B_1_1 = 1.0f/Ls;
B_2_0 = 0.0f;
B_2_1 = 0.0f;
B_3_0 = 0.0f;
B_3_1 = 0.0f;

P0_0_0 = 0.0f;
P0_0_1 = 0.0f;
P0_0_2 = 0.0f;
P0_0_3 = 0.0f;
P0_1_0 = 0.0f;
P0_1_1 = 0.0f;
P0_1_2 = 0.0f;
P0_1_3 = 0.0f;
P0_2_0 = 0.0f;
P0_2_1 = 0.0f;
P0_2_2 = 0.0f;
P0_2_3 = 0.0f;
P0_3_0 = 0.0f;
P0_3_1 = 0.0f;
P0_3_2 = 0.0f;
P0_3_3 = 0.0f;
/* %%%-SFUNWIZ_wrapper_Start_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Output function
 *
 */
void stm32_ekf_Outputs_wrapper(const real32_T *u,
			real32_T *y,
			const real_T *xD)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
/**********************************
             ??????
        ?????????QQ????B?
          QQ??1323279545
        ??????www.huiqudong.cn
**********************************/
y[0] = xD[0];
y[1] = xD[1];
y[2] = xD[2];
y[3] = xD[3];
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}

/*
 * Updates function
 *
 */
void stm32_ekf_Update_wrapper(const real32_T *u,
			real32_T *y,
			real_T *xD)
{
/* %%%-SFUNWIZ_wrapper_Update_Changes_BEGIN --- EDIT HERE TO _END */
/**********************************
             ??????
        ?????????QQ????B?
          QQ??1323279545
        ??????www.huiqudong.cn
**********************************/



vs_ab_0_0 = u[0];
vs_ab_1_0 = u[1];
is_ab_0_0 = u[2];
is_ab_1_0 = u[3];

Rs = u[4];//????
Ls = u[5];//???
flux = u[6];//????
/*
P0_0_0 = xD[4];
P0_0_1 = xD[5];
P0_0_2 = xD[6];
P0_0_3 = xD[7];
P0_1_0 = xD[8];
P0_1_1 = xD[9];
P0_1_2 = xD[10];
P0_1_3 = xD[11];
P0_2_0 = xD[12];
P0_2_1 = xD[13];
P0_2_2 = xD[14];
P0_2_3 = xD[15];
P0_3_0 = xD[16];
P0_3_1 = xD[17];
P0_3_2 = xD[18];
P0_3_3 = xD[19];
*/
#ifdef  SIMULINK_USE_ARM_MATH  
F_0_0 = -Rs/Ls;
F_0_1 = 0.0f;
F_0_2 = flux/Ls*arm_sin_f32(xD[3]);
F_0_3 = flux/Ls*xD[2]*arm_cos_f32(xD[3]);
F_1_0 = 0.0f;
F_1_1 = -Rs/Ls;
F_1_2 = -flux/Ls*arm_cos_f32(xD[3]);
F_1_3 = flux/Ls*xD[2]*arm_sin_f32(xD[3]);
F_2_0 = 0.0f;
F_2_1 = 0.0f;
F_2_2 = 0.0f;
F_2_3 = 0.0f;
F_3_0 = 0.0f;
F_3_1 = 0.0f;
F_3_2 = 1.0f;
F_3_3 = 0.0f;
#else
F_0_0 = -Rs/Ls;
F_0_1 = 0.0f;
F_0_2 = flux/Ls*sin(xD[3]);
F_0_3 = flux/Ls*xD[2]*cos(xD[3]);
F_1_0 = 0.0f;
F_1_1 = -Rs/Ls;
F_1_2 = -flux/Ls*cos(xD[3]);
F_1_3 = flux/Ls*xD[2]*sin(xD[3]);
F_2_0 = 0.0f;
F_2_1 = 0.0f;
F_2_2 = 0.0f;
F_2_3 = 0.0f;
F_3_0 = 0.0f;
F_3_1 = 0.0f;
F_3_2 = 1.0f;
F_3_3 = 0.0f;
#endif
temp_0_0 = 1.0f;
temp_0_1 = 0.0f;
temp_0_2 = 0.0f;
temp_0_3 = 0.0f;
temp_1_0 = 0.0f;
temp_1_1 = 1.0f;
temp_1_2 = 0.0f;
temp_1_3 = 0.0f;
temp_2_0 = 0.0f;
temp_2_1 = 0.0f;
temp_2_2 = 1.0f;
temp_2_3 = 0.0f;
temp_3_0 = 0.0f;
temp_3_1 = 0.0f;
temp_3_2 = 0.0f;
temp_3_3 = 1.0f;

#ifdef  SIMULINK_USE_ARM_MATH 
f1_0_0 = -Rs/Ls*xD[0]+flux/Ls*xD[2]*arm_sin_f32(xD[3]);
f1_1_0 = -Rs/Ls*xD[1]-flux/Ls*xD[2]*arm_cos_f32(xD[3]);
f1_2_0 = 0.0f;
f1_3_0 = xD[2];
#else
f1_0_0 = -Rs/Ls*xD[0]+flux/Ls*xD[2]*sin(xD[3]);
f1_1_0 = -Rs/Ls*xD[1]-flux/Ls*xD[2]*cos(xD[3]);
f1_2_0 = 0.0f;
f1_3_0 = xD[2];
#endif


f2_0_0 = temp_0_0 + (T*F_0_0);
f2_0_1 = temp_0_1 + (T*F_0_1);
f2_0_2 = temp_0_2 + (T*F_0_2);
f2_0_3 = temp_0_3 + (T*F_0_3);
f2_1_0 = temp_1_0 + (T*F_1_0);
f2_1_1 = temp_1_1 + (T*F_1_1);
f2_1_2 = temp_1_2 + (T*F_1_2);
f2_1_3 = temp_1_3 + (T*F_1_3);
f2_2_0 = temp_2_0 + (T*F_2_0);
f2_2_1 = temp_2_1 + (T*F_2_1);
f2_2_2 = temp_2_2 + (T*F_2_2);
f2_2_3 = temp_2_3 + (T*F_2_3);
f2_3_0 = temp_3_0 + (T*F_3_0);
f2_3_1 = temp_3_1 + (T*F_3_1);
f2_3_2 = temp_3_2 + (T*F_3_2);
f2_3_3 = temp_3_3 + (T*F_3_3);

X_pred_0_0 = xD[0] + T*(f1_0_0 + B_0_0*vs_ab_0_0 + B_0_1*vs_ab_1_0);
X_pred_1_0 = xD[1] + T*(f1_1_0 + B_1_0*vs_ab_0_0 + B_1_1*vs_ab_1_0);
X_pred_2_0 = xD[2] + T*(f1_2_0 + B_2_0*vs_ab_0_0 + B_2_1*vs_ab_1_0);
X_pred_3_0 = xD[3] + T*(f1_3_0 + B_3_0*vs_ab_0_0 + B_3_1*vs_ab_1_0);

Y_pred_0_0 = H_0_0*X_pred_0_0 + H_0_1*X_pred_1_0 + H_0_2*X_pred_2_0 + H_0_3*X_pred_3_0;
Y_pred_1_0 = H_1_0*X_pred_0_0 + H_1_1*X_pred_1_0 + H_1_2*X_pred_2_0 + H_1_3*X_pred_3_0;


Y_0_0 = is_ab_0_0;
Y_1_0 = is_ab_1_0;

P_pred_0_0 = f2_0_0*P0_0_0 + f2_0_1*P0_1_0 + f2_0_2*P0_2_0 + f2_0_3*P0_3_0;
P_pred_0_1 = f2_0_0*P0_0_1 + f2_0_1*P0_1_1 + f2_0_2*P0_2_1 + f2_0_3*P0_3_1;
P_pred_0_2 = f2_0_0*P0_0_2 + f2_0_1*P0_1_2 + f2_0_2*P0_2_2 + f2_0_3*P0_3_2;
P_pred_0_3 = f2_0_0*P0_0_3 + f2_0_1*P0_1_3 + f2_0_2*P0_2_3 + f2_0_3*P0_3_3;    
P_pred_1_0 = f2_1_0*P0_0_0 + f2_1_1*P0_1_0 + f2_1_2*P0_2_0 + f2_1_3*P0_3_0;
P_pred_1_1 = f2_1_0*P0_0_1 + f2_1_1*P0_1_1 + f2_1_2*P0_2_1 + f2_1_3*P0_3_1;
P_pred_1_2 = f2_1_0*P0_0_2 + f2_1_1*P0_1_2 + f2_1_2*P0_2_2 + f2_1_3*P0_3_2;
P_pred_1_3 = f2_1_0*P0_0_3 + f2_1_1*P0_1_3 + f2_1_2*P0_2_3 + f2_1_3*P0_3_3;  
P_pred_2_0 = f2_2_0*P0_0_0 + f2_2_1*P0_1_0 + f2_2_2*P0_2_0 + f2_2_3*P0_3_0;
P_pred_2_1 = f2_2_0*P0_0_1 + f2_2_1*P0_1_1 + f2_2_2*P0_2_1 + f2_2_3*P0_3_1;
P_pred_2_2 = f2_2_0*P0_0_2 + f2_2_1*P0_1_2 + f2_2_2*P0_2_2 + f2_2_3*P0_3_2;
P_pred_2_3 = f2_2_0*P0_0_3 + f2_2_1*P0_1_3 + f2_2_2*P0_2_3 + f2_2_3*P0_3_3;
P_pred_3_0 = f2_3_0*P0_0_0 + f2_3_1*P0_1_0 + f2_3_2*P0_2_0 + f2_3_3*P0_3_0;
P_pred_3_1 = f2_3_0*P0_0_1 + f2_3_1*P0_1_1 + f2_3_2*P0_2_1 + f2_3_3*P0_3_1;
P_pred_3_2 = f2_3_0*P0_0_2 + f2_3_1*P0_1_2 + f2_3_2*P0_2_2 + f2_3_3*P0_3_2;
P_pred_3_3 = f2_3_0*P0_0_3 + f2_3_1*P0_1_3 + f2_3_2*P0_2_3 + f2_3_3*P0_3_3;

P_pred_0_0 = P_pred_0_0*f2_0_0 + P_pred_0_1*f2_0_1 + P_pred_0_2*f2_0_2 + P_pred_0_3*f2_0_3 + Q_0_0;
P_pred_0_1 = P_pred_0_0*f2_1_0 + P_pred_0_1*f2_1_1 + P_pred_0_2*f2_1_2 + P_pred_0_3*f2_1_3 + Q_0_1;
P_pred_0_2 = P_pred_0_0*f2_2_0 + P_pred_0_1*f2_2_1 + P_pred_0_2*f2_2_2 + P_pred_0_3*f2_2_3 + Q_0_2;
P_pred_0_3 = P_pred_0_0*f2_3_0 + P_pred_0_1*f2_3_1 + P_pred_0_2*f2_3_2 + P_pred_0_3*f2_3_3 + Q_0_3;    
P_pred_1_0 = P_pred_1_0*f2_0_0 + P_pred_1_1*f2_0_1 + P_pred_1_2*f2_0_2 + P_pred_1_3*f2_0_3 + Q_1_0;
P_pred_1_1 = P_pred_1_0*f2_1_0 + P_pred_1_1*f2_1_1 + P_pred_1_2*f2_1_2 + P_pred_1_3*f2_1_3 + Q_1_1;
P_pred_1_2 = P_pred_1_0*f2_2_0 + P_pred_1_1*f2_2_1 + P_pred_1_2*f2_2_2 + P_pred_1_3*f2_2_3 + Q_1_2;
P_pred_1_3 = P_pred_1_0*f2_3_0 + P_pred_1_1*f2_3_1 + P_pred_1_2*f2_3_2 + P_pred_1_3*f2_3_3 + Q_1_3;    
P_pred_2_0 = P_pred_2_0*f2_0_0 + P_pred_2_1*f2_0_1 + P_pred_2_2*f2_0_2 + P_pred_2_3*f2_0_3 + Q_2_0;
P_pred_2_1 = P_pred_2_0*f2_1_0 + P_pred_2_1*f2_1_1 + P_pred_2_2*f2_1_2 + P_pred_2_3*f2_1_3 + Q_2_1;
P_pred_2_2 = P_pred_2_0*f2_2_0 + P_pred_2_1*f2_2_1 + P_pred_2_2*f2_2_2 + P_pred_2_3*f2_2_3 + Q_2_2;
P_pred_2_3 = P_pred_2_0*f2_3_0 + P_pred_2_1*f2_3_1 + P_pred_2_2*f2_3_2 + P_pred_2_3*f2_3_3 + Q_2_3; 
P_pred_3_0 = P_pred_3_0*f2_0_0 + P_pred_3_1*f2_0_1 + P_pred_3_2*f2_0_2 + P_pred_3_3*f2_0_3 + Q_3_0;
P_pred_3_1 = P_pred_3_0*f2_1_0 + P_pred_3_1*f2_1_1 + P_pred_3_2*f2_1_2 + P_pred_3_3*f2_1_3 + Q_3_1;
P_pred_3_2 = P_pred_3_0*f2_2_0 + P_pred_3_1*f2_2_1 + P_pred_3_2*f2_2_2 + P_pred_3_3*f2_2_3 + Q_3_2;
P_pred_3_3 = P_pred_3_0*f2_3_0 + P_pred_3_1*f2_3_1 + P_pred_3_2*f2_3_2 + P_pred_3_3*f2_3_3 + Q_3_3;

    temp_0_0 = H_0_0*P_pred_0_0 + H_0_1*P_pred_1_0 + H_0_2*P_pred_2_0 + H_0_3*P_pred_3_0;
    temp_0_1 = H_0_0*P_pred_0_1 + H_0_1*P_pred_1_1 + H_0_2*P_pred_2_1 + H_0_3*P_pred_3_1;
    temp_0_2 = H_0_0*P_pred_0_2 + H_0_1*P_pred_1_2 + H_0_2*P_pred_2_2 + H_0_3*P_pred_3_2;
    temp_0_3 = H_0_0*P_pred_0_3 + H_0_1*P_pred_1_3 + H_0_2*P_pred_2_3 + H_0_3*P_pred_3_3;   
    temp_1_0 = H_1_0*P_pred_0_0 + H_1_1*P_pred_1_0 + H_1_2*P_pred_2_0 + H_1_3*P_pred_3_0;
    temp_1_1 = H_1_0*P_pred_0_1 + H_1_1*P_pred_1_1 + H_1_2*P_pred_2_1 + H_1_3*P_pred_3_1;
    temp_1_2 = H_1_0*P_pred_0_2 + H_1_1*P_pred_1_2 + H_1_2*P_pred_2_2 + H_1_3*P_pred_3_2;
    temp_1_3 = H_1_0*P_pred_0_3 + H_1_1*P_pred_1_3 + H_1_2*P_pred_2_3 + H_1_3*P_pred_3_3;


    temp_0_0 = temp_0_0*H_0_0 + temp_0_1*H_0_1 + temp_0_2*H_0_2 + temp_0_3*H_0_3 + R_0_0;
    temp_0_1 = temp_0_0*H_1_0 + temp_0_1*H_1_1 + temp_0_2*H_1_2 + temp_0_3*H_1_3 + R_0_1;    
    temp_1_0 = temp_1_0*H_0_0 + temp_1_1*H_0_1 + temp_1_2*H_0_2 + temp_1_3*H_0_3 + R_1_0;
    temp_1_1 = temp_1_0*H_1_0 + temp_1_1*H_1_1 + temp_1_2*H_1_2 + temp_1_3*H_1_3 + R_1_1;

temp_0_0_t = temp_0_0;
temp_0_1_t = temp_0_1;
temp_1_0_t = temp_1_0;
temp_1_1_t = temp_1_1;

temp = temp_0_0*temp_1_1 - temp_0_1*temp_1_0;
if(temp != 0)
{
   temp_0_0 = temp_1_1_t/temp;
   temp_0_1 = -temp_0_1_t/temp;
   temp_1_0 = -temp_1_0_t/temp;
   temp_1_1 = temp_0_0_t/temp;
}

K_0_0 = P_pred_0_0*H_0_0 + P_pred_0_1*H_0_1 + P_pred_0_2*H_0_2 + P_pred_0_3*H_0_3;
K_0_1 = P_pred_0_0*H_1_0 + P_pred_0_1*H_1_1 + P_pred_0_2*H_1_2 + P_pred_0_3*H_1_3;    
K_1_0 = P_pred_1_0*H_0_0 + P_pred_1_1*H_0_1 + P_pred_1_2*H_0_2 + P_pred_1_3*H_0_3;
K_1_1 = P_pred_1_0*H_1_0 + P_pred_1_1*H_1_1 + P_pred_1_2*H_1_2 + P_pred_1_3*H_1_3;   
K_2_0 = P_pred_2_0*H_0_0 + P_pred_2_1*H_0_1 + P_pred_2_2*H_0_2 + P_pred_2_3*H_0_3;
K_2_1 = P_pred_2_0*H_1_0 + P_pred_2_1*H_1_1 + P_pred_2_2*H_1_2 + P_pred_2_3*H_1_3;   
K_3_0 = P_pred_3_0*H_0_0 + P_pred_3_1*H_0_1 + P_pred_3_2*H_0_2 + P_pred_3_3*H_0_3;
K_3_1 = P_pred_3_0*H_1_0 + P_pred_3_1*H_1_1 + P_pred_3_2*H_1_2 + P_pred_3_3*H_1_3;



K_0_0 = K_0_0*temp_0_0 + K_0_1*temp_1_0;
K_0_1 = K_0_0*temp_0_1 + K_0_1*temp_1_1;  
K_1_0 = K_1_0*temp_0_0 + K_1_1*temp_1_0;
K_1_1 = K_1_0*temp_0_1 + K_1_1*temp_1_1;    
K_2_0 = K_2_0*temp_0_0 + K_2_1*temp_1_0;
K_2_1 = K_2_0*temp_0_1 + K_2_1*temp_1_1;   
K_3_0 = K_3_0*temp_0_0 + K_3_1*temp_1_0;
K_3_1 = K_3_0*temp_0_1 + K_3_1*temp_1_1;

tempa_0_0 = X_pred_0_0 + K_0_0*(Y_0_0 - Y_pred_0_0) + K_0_1*(Y_1_0 - Y_pred_1_0);
tempa_1_0 = X_pred_1_0 + K_1_0*(Y_0_0 - Y_pred_0_0) + K_1_1*(Y_1_0 - Y_pred_1_0);
tempa_2_0 = X_pred_2_0 + K_2_0*(Y_0_0 - Y_pred_0_0) + K_2_1*(Y_1_0 - Y_pred_1_0);
tempa_3_0 = X_pred_3_0 + K_3_0*(Y_0_0 - Y_pred_0_0) + K_3_1*(Y_1_0 - Y_pred_1_0);

    temp_0_0 = K_0_0*H_0_0 + K_0_1*H_1_0;
    temp_0_1 = K_0_0*H_0_1 + K_0_1*H_1_1;
    temp_0_2 = K_0_0*H_0_2 + K_0_1*H_1_2;
    temp_0_3 = K_0_0*H_0_3 + K_0_1*H_1_3;    
    temp_1_0 = K_1_0*H_0_0 + K_1_1*H_1_0;
    temp_1_1 = K_1_0*H_0_1 + K_1_1*H_1_1;
    temp_1_2 = K_1_0*H_0_2 + K_1_1*H_1_2;
    temp_1_3 = K_1_0*H_0_3 + K_1_1*H_1_3;    
    temp_2_0 = K_2_0*H_0_0 + K_2_1*H_1_0;
    temp_2_1 = K_2_0*H_0_1 + K_2_1*H_1_1;
    temp_2_2 = K_2_0*H_0_2 + K_2_1*H_1_2;
    temp_2_3 = K_2_0*H_0_3 + K_2_1*H_1_3;   
    temp_3_0 = K_3_0*H_0_0 + K_3_1*H_1_0;
    temp_3_1 = K_3_0*H_0_1 + K_3_1*H_1_1;
    temp_3_2 = K_3_0*H_0_2 + K_3_1*H_1_2;
   temp_3_3 = K_3_0*H_0_3 + K_3_1*H_1_3;


   
   
    P0_0_0 =   P_pred_0_0 - (temp_0_0*P_pred_0_0 + temp_0_1*P_pred_1_0 + temp_0_2*P_pred_2_0 + temp_0_3*P_pred_3_0);
    P0_0_1 =   P_pred_0_1 - (temp_0_0*P_pred_0_1 + temp_0_1*P_pred_1_1 + temp_0_2*P_pred_2_1 + temp_0_3*P_pred_3_1);
    P0_0_2 =   P_pred_0_2 - (temp_0_0*P_pred_0_2 + temp_0_1*P_pred_1_2 + temp_0_2*P_pred_2_2 + temp_0_3*P_pred_3_2);
    P0_0_3 =   P_pred_0_3 - (temp_0_0*P_pred_0_3 + temp_0_1*P_pred_1_3 + temp_0_2*P_pred_2_3 + temp_0_3*P_pred_3_3);  
    P0_1_0 =   P_pred_1_0 - (temp_1_0*P_pred_0_0 + temp_1_1*P_pred_1_0 + temp_1_2*P_pred_2_0 + temp_1_3*P_pred_3_0);
    P0_1_1 =   P_pred_1_1 - (temp_1_0*P_pred_0_1 + temp_1_1*P_pred_1_1 + temp_1_2*P_pred_2_1 + temp_1_3*P_pred_3_1);
    P0_1_2 =   P_pred_1_2 - (temp_1_0*P_pred_0_2 + temp_1_1*P_pred_1_2 + temp_1_2*P_pred_2_2 + temp_1_3*P_pred_3_2);
    P0_1_3 =   P_pred_1_3 - (temp_1_0*P_pred_0_3 + temp_1_1*P_pred_1_3 + temp_1_2*P_pred_2_3 + temp_1_3*P_pred_3_3);    
    P0_2_0 =   P_pred_2_0 - (temp_2_0*P_pred_0_0 + temp_2_1*P_pred_1_0 + temp_2_2*P_pred_2_0 + temp_2_3*P_pred_3_0);
    P0_2_1 =   P_pred_2_1 - (temp_2_0*P_pred_0_1 + temp_2_1*P_pred_1_1 + temp_2_2*P_pred_2_1 + temp_2_3*P_pred_3_1);
    P0_2_2 =   P_pred_2_2 - (temp_2_0*P_pred_0_2 + temp_2_1*P_pred_1_2 + temp_2_2*P_pred_2_2 + temp_2_3*P_pred_3_2);
    P0_2_3 =   P_pred_2_3 - (temp_2_0*P_pred_0_3 + temp_2_1*P_pred_1_3 + temp_2_2*P_pred_2_3 + temp_2_3*P_pred_3_3);  
    P0_3_0 =   P_pred_3_0 - (temp_3_0*P_pred_0_0 + temp_3_1*P_pred_1_0 + temp_3_2*P_pred_2_0 + temp_3_3*P_pred_3_0);
    P0_3_1 =   P_pred_3_1 - (temp_3_0*P_pred_0_1 + temp_3_1*P_pred_1_1 + temp_3_2*P_pred_2_1 + temp_3_3*P_pred_3_1);
    P0_3_2 =   P_pred_3_2 - (temp_3_0*P_pred_0_2 + temp_3_1*P_pred_1_2 + temp_3_2*P_pred_2_2 + temp_3_3*P_pred_3_2);
    P0_3_3 =   P_pred_3_3 - (temp_3_0*P_pred_0_3 + temp_3_1*P_pred_1_3 + temp_3_2*P_pred_2_3 + temp_3_3*P_pred_3_3);
#ifdef  SIMULINK_USE_ARM_MATH
    if(tempa_3_0>(2.0f*PI))
    {
       tempa_3_0 -= (2.0f*PI);
    }
    #else
        if(tempa_3_0>(6.2831853f))
    {
       tempa_3_0 -= (6.2831853f);
    }

    #endif
xD[0] = tempa_0_0;
xD[1] = tempa_1_0;
xD[2] = tempa_2_0;
xD[3] = tempa_3_0;
/*
xD[4] = P0_0_0;
xD[5] = P0_0_1;
xD[6] = P0_0_2;
xD[7] = P0_0_3;
xD[8] = P0_1_0;
xD[9] = P0_1_1;
xD[10] = P0_1_2;
xD[11] = P0_1_3;
xD[12] = P0_2_0;
xD[13] = P0_2_1;
xD[14] = P0_2_2;
xD[15] = P0_2_3;
xD[16] = P0_3_0;
xD[17] = P0_3_1;
xD[18] = P0_3_2;
xD[19] = P0_3_3;*/
/* %%%-SFUNWIZ_wrapper_Update_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Terminate function
 *
 */
void stm32_ekf_Terminate_wrapper(real_T *xD)
{
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_BEGIN --- EDIT HERE TO _END */
/*
 * Custom Terminate code goes here.
 */
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_END --- EDIT HERE TO _BEGIN */
}

