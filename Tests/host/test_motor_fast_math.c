/* 快速数学：sincos 精度、Park/反 Park 与二维矢量限幅对参考实现核对。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <float.h>
#include "motor_foc.h"
#include "motor_math.h"
/**
 * @description: 计算测试比较用的参考结果。
 * @param {float *} x x 参数
 * @param {float *} y y 参数
 * @param {float} limit limit 参数
 * @return {bool} 本次处理的判定结果，具体条件见职责与实现说明
 */
static bool reference(float *x,float *y,float limit) {
 float mag=sqrtf(*x * *x + *y * *y);limit=fabsf(limit);
 if(mag<1e-10f)mag=1e-10f;
 if(mag>limit){float f=limit/mag;*x*=f;*y*=f;return true;}
 return false;
}
/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void) {
 unsigned i,j; float max_trig=0,max_dq=0;
 {
  float maximum=0.0f,amplitude_maximum=0.0f;
  for(i=0;i<=262144U;i++) {
   float angle=-64.0f+128.0f*(float)i/262144.0f,sine,cosine,error,amplitude_error;
   sincos_taylor_f32(angle,&sine,&cosine);
   error=fmaxf(fabsf(sine-sinf(angle)),fabsf(cosine-cosf(angle)));
   amplitude_error=fabsf(sine*sine+cosine*cosine-1.0f);
   if(error>maximum)maximum=error;
   if(amplitude_error>amplitude_maximum)amplitude_maximum=amplitude_error;
   assert(error<0.000001f && amplitude_error<0.000002f);
  }
  for(i=0;i<81U;i++) {
   float boundary=((float)i-40.0f)*((float)M_PI/4.0f);
   float angles[]={nextafterf(boundary,-INFINITY),boundary,nextafterf(boundary,INFINITY)};
   for(j=0;j<3U;j++) {
    float sine,cosine;
    sincos_taylor_f32(angles[j],&sine,&cosine);
    assert(fabsf(sine-sinf(angles[j]))<0.000001f);
    assert(fabsf(cosine-cosf(angles[j]))<0.000001f);
   }
  }
  { float sine,cosine; sincos_taylor_f32(-0.0f,&sine,&cosine); assert(signbit(sine)&&cosine==1.0f); }
  printf("Taylor math: standard max=%g unit-circle max=%g\n",maximum,amplitude_maximum);
 }
 {
  const float special[]={NAN,INFINITY,-INFINITY,FLT_MAX,-FLT_MAX,65.0f,-65.0f,1000000.0f};
  for(i=0;i<sizeof(special)/sizeof(special[0]);i++) {
   float sine,cosine,expected_sine=sinf(special[i]),expected_cosine=cosf(special[i]);
   sincos_f32(special[i],&sine,&cosine);
   assert((isnan(sine)&&isnan(expected_sine))||sine==expected_sine);
   assert((isnan(cosine)&&isnan(expected_cosine))||cosine==expected_cosine);
   sincos_taylor_f32(special[i],&sine,&cosine);
   assert((isnan(sine)&&isnan(expected_sine))||sine==expected_sine);
   assert((isnan(cosine)&&isnan(expected_cosine))||cosine==expected_cosine);
  }
  { float sine,cosine; sincos_f32(-0.0f,&sine,&cosine); assert(signbit(sine)&&cosine==1.0f); }
  for(i=0;i<=131072U;i++) {
   float angle=-64.0f+128.0f*(float)i/131072.0f,sine,cosine;
   sincos_f32(angle,&sine,&cosine);
   assert(fabsf(sine-sinf(angle))<0.000025f);
   assert(fabsf(cosine-cosf(angle))<0.000025f);
  }
 }
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
