/*
 * @Description: 
 * @Date: 2023-02-18 23:29:45
 * @LastEditTime: 2023-03-26 20:05:11
 * @FilePath: \foc\App\motor\inc\utils.h
 */
#ifndef _MOTOR_FOC_H
#define _MOTOR_FOC_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * A simple low pass filter.
 *
 * @param value
 * The filtered value.
 *
 * @param sample
 * Next sample.
 *
 * @param filter_constant
 * Filter constant. Range 0.0 to 1.0, where 1.0 gives the unfiltered value.
 */
#define UTILS_LP_FAST(value, sample, filter_constant)    (value -= (filter_constant) * ((value) - (sample)))

/**
 * A fast approximation of a moving average filter with N samples. See
 * https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average
 * https://en.wikipedia.org/wiki/Exponential_smoothing
 *
 * It is not entirely the same as it behaves like an IIR filter rather than a FIR filter, but takes
 * much less memory and is much faster to run.
 */
//#define UTILS_LP_MOVING_AVG_APPROX(value, sample, N)    UTILS_LP_FAST(value, sample, 2.0f / ((N) + 1.0f))

extern inline void clarke_transform(float Ia, float Ib, float Ic, float *Ialpha, float *Ibeta);
extern inline void park_transform(float Ialpha, float Ibeta, float Theta, float *Id, float *Iq);
extern inline void inverse_park(float mod_d, float mod_q, float Theta, float *mod_alpha, float *mod_beta);
extern inline int svm(float alpha, float beta, float* tA, float* tB, float* tC);
void foc_svm(float alpha, float beta, uint32_t PWMFullDutyCycle,
				uint32_t* tAout, uint32_t* tBout, uint32_t* tCout, uint32_t *svm_sector);

float sin_f32(float x);
float cos_f32(float x);

uint8_t crc8(const uint8_t *data, const uint32_t size);
uint32_t crc32(const uint8_t *data, uint32_t size);

int uint32_to_data(uint32_t val, uint8_t *data);
int int32_to_data(int32_t val, uint8_t *data);
int uint16_to_data(uint16_t val, uint8_t *data);
int int16_to_data(int16_t val, uint8_t *data);
int float_to_data(float val, uint8_t *data);

uint32_t data_to_uint32(uint8_t *data);
int32_t data_to_int32(uint8_t *data);
uint16_t data_to_uint16(uint8_t *data);
int16_t data_to_int16(uint8_t *data);
float data_to_float(uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif
