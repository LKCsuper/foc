#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include "motor_foc.h"
#include "motor_math.h"
static bool reference(float *x,float *y,float limit) {
 float mag=sqrtf(*x * *x + *y * *y);limit=fabsf(limit);
 if(mag<1e-10f)mag=1e-10f;
 if(mag>limit){float f=limit/mag;*x*=f;*y*=f;return true;}
 return false;
}
int main(void) {
 unsigned i,j; float max_trig=0,max_dq=0;
 for(i=0;i<=65536;i++) {
  float angle=((float)i/65536.0f*4.0f-2.0f)*M_PI,s,c,old_s=sin_f32(angle),old_c=cos_f32(angle);
  float d,q,a,b,rd,rq;
  sincos_f32(angle,&s,&c);
  float error=fmaxf(fabsf(s-old_s),fabsf(c-old_c));
  if(error>max_trig)max_trig=error;
  assert(error<1e-6f);
  park_transform(.7f,-.3f,angle,&d,&q);
  rd=.7f*old_c-.3f*old_s;rq=-.7f*old_s-.3f*old_c;
  error=fmaxf(fabsf(d-rd),fabsf(q-rq));if(error>max_dq)max_dq=error;
  assert(error<1e-6f);
  inverse_park(3.0f,-2.0f,angle,&a,&b);
  assert(fabsf(a+(3.0f*old_c+2.0f*old_s))<4e-6f);
  assert(fabsf(b+(3.0f*old_s-2.0f*old_c))<4e-6f);
 }
 const float limits[]={0,1e-12f,1e-10f,.01f,1,4,-4,1e30f,INFINITY,NAN};
 for(j=0;j<sizeof(limits)/sizeof(limits[0]);j++) {
  for(i=0;i<20000;i++) {
   float t=(float)i*.013f,x=cosf(t)*(float)(i%101)*.1f,y=sinf(t)*(float)(i%107)*.1f;
   float rx=x,ry=y;bool r=reference(&rx,&ry,limits[j]),actual=utils_saturate_vector_2d(&x,&y,limits[j]);
   assert(r==actual);assert((isnan(x)&&isnan(rx))||x==rx);assert((isnan(y)&&isnan(ry))||y==ry);
  }
 }
 for(i=0;i<10000;i++) {
  float x=nextafterf(4.0f,i&1 ? INFINITY:0),y=(float)(i%3)*1e-7f,rx=x,ry=y;
  bool r=reference(&rx,&ry,4.0f),actual=utils_saturate_vector_2d(&x,&y,4.0f);
  assert(r==actual && x==rx && y==ry);
 }
 printf("fast math: trig max=%g dq max=%g, limiter 210000 reference cases pass\n",max_trig,max_dq);
 return 0;
}
