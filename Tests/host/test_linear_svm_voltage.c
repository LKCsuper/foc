#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "motor_foc.h"

/* 独立三相平均电压方程核对真实调制实现，不使用输出函数桩。 */
int main(void)
{
    unsigned k;
    float maximum = 0.0f;
    const float buses[] = {16.0f, 24.0f};
    for (k = 0; k < 8192; ++k) {
        unsigned bus_index = k / 4096U;
        uint32_t a, b, c, sector;
        float theta = (k % 4096U) * 6.283185307f / 4096;
        float va = 3.0f * cosf(theta), vb = 3.0f * sinf(theta);
        float reconstructed_a, reconstructed_b, error;
        foc_svm_from_voltage(va, vb, buses[bus_index], 0.95f, 5600, &a, &b, &c, &sector);
        reconstructed_a = (2.0f * a - b - c) * buses[bus_index] / (3.0f * 5600);
        reconstructed_b = ((float)b - c) * buses[bus_index] / (1.732050808f * 5600);
        error = hypotf(reconstructed_a - va, reconstructed_b - vb);
        if (error > maximum) maximum = error;
        assert(error < 0.006f);
    }
    printf("linear SVM 16/24V, 3V sweep: max reconstruction error %.6fV; gain is 1, not 0.75\n", maximum);
    return 0;
}
