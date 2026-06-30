
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



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */

#include <math.h>
float theta0_1_1;
float theta0_2_1;
float Pn0_1_1;
float Pn0_1_2;
float Pn0_2_1;
float Pn0_2_2;
float x_1_1;
float x_1_2;
float x_1_3;
float x_2_1;
float x_2_2;
float x_2_3;
float h_1_1;
float h_2_1;
float r_K_1_1;
float r_K_2_1;
float Pn1_1_1;
float Pn1_1_2;
float Pn1_2_1;
float Pn1_2_2;
float theta1_1_1;
float theta1_2_1;
float r_temp_1_1;
float r_temp_1_2;
float r_temp_2_1;
float r_temp_2_2;
float r_Uq;
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 3
#define y_width 2

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
void R_flux_identification_Start_wrapper(real_T *xD)
{
/* %%%-SFUNWIZ_wrapper_Start_Changes_BEGIN --- EDIT HERE TO _END */

/*
 * Custom Start code goes here.
 */

theta0_1_1 = 0.05f;
theta0_2_1 = 0.01f;

Pn0_1_1 = 0.0008f*2.0f;
Pn0_1_2 = 0.0008f*0.0f;
Pn0_2_1 = 0.0008f*0.0f;
Pn0_2_2 = 0.0008f*2.0f;

x_1_1 = theta0_1_1;
x_2_1 = theta0_2_1;
x_1_2 = Pn0_1_1;
x_1_3 = Pn0_1_2;
x_2_2 = Pn0_2_1;
x_2_3 = Pn0_2_2;
/* %%%-SFUNWIZ_wrapper_Start_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Output function
 *
 */
void R_flux_identification_Outputs_wrapper(const real32_T *u,
			real32_T *y,
			const real_T *xD)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */

/* This sample sets the output equal to the input
      y0[0] = u0[0]; 
 For complex signals use: y0[0].re = u0[0].re; 
      y0[0].im = u0[0].im;
      y1[0].re = u1[0].re;
      y1[0].im = u1[0].im;
 */

y[0] = x_1_1;
y[1] = x_2_1;
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}

/*
 * Updates function
 *
 */
void R_flux_identification_Update_wrapper(const real32_T *u,
			real32_T *y,
			real_T *xD)
{
/* %%%-SFUNWIZ_wrapper_Update_Changes_BEGIN --- EDIT HERE TO _END */

/*
 * Code example
 *   xD[0] = u0[0];
 */

h_1_1 = u[0];
h_2_1 = u[1];
r_Uq = u[2];
Pn0_1_1 = x_1_2;
Pn0_1_2 = x_1_3;
Pn0_2_1 = x_2_2;
Pn0_2_2 = x_2_3;

r_temp_1_1 = 1.0f + (((h_1_1*Pn0_1_1 + h_2_1* Pn0_2_1)*h_1_1) + ((h_1_1*Pn0_1_2 + h_2_1* Pn0_2_2)*h_2_1));
r_temp_1_1 = 1.0f/r_temp_1_1;
r_K_1_1 = (Pn0_1_1*h_1_1+Pn0_1_2*h_2_1)*r_temp_1_1;
r_K_2_1 = (Pn0_2_1*h_1_1+Pn0_2_2*h_2_1)*r_temp_1_1;

r_temp_1_1 = r_K_1_1*h_1_1;
r_temp_1_2 = r_K_1_1*h_2_1;
r_temp_2_1 = r_K_2_1*h_1_1;
r_temp_2_2 = r_K_2_1*h_2_1;
Pn1_1_1 = Pn0_1_1 - (r_temp_1_1*Pn0_1_1+r_temp_1_2*Pn0_2_1);
Pn1_1_2 = Pn0_1_2 - (r_temp_1_1*Pn0_1_2+r_temp_1_2*Pn0_2_2);
Pn1_2_1 = Pn0_2_1 - (r_temp_2_1*Pn0_1_1+r_temp_2_2*Pn0_2_1);
Pn1_2_2 = Pn0_2_2 - (r_temp_2_1*Pn0_1_2+r_temp_2_2*Pn0_2_2);

theta0_1_1 = x_1_1;
theta0_2_1 = x_2_1;

theta1_1_1 = theta0_1_1 + r_K_1_1*(r_Uq - (h_1_1*theta0_1_1*0.999f+h_2_1*theta0_2_1*0.999f));
theta1_2_1 = theta0_2_1 + r_K_2_1*(r_Uq - (h_1_1*theta0_1_1*0.999f+h_2_1*theta0_2_1*0.999f));

x_1_1 = theta1_1_1;
x_2_1 = theta1_2_1;
x_1_2 = Pn1_1_1;
x_1_3 = Pn1_1_2;
x_2_2 = Pn1_2_1;
x_2_3 = Pn1_2_2;
/* %%%-SFUNWIZ_wrapper_Update_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Terminate function
 *
 */
void R_flux_identification_Terminate_wrapper(real_T *xD)
{
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_BEGIN --- EDIT HERE TO _END */
/*
 * Custom Terminate code goes here.
 */
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_END --- EDIT HERE TO _BEGIN */
}

