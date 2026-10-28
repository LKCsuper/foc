/* SVM 电压重建：三相平均电压方程核对真实调制，16/24 V 母线全角扫描。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "motor_foc.h"

/**
 * @description: 按几何扇区及两种调制的相反极性核对共享扇区选择。
 * @return {void} 无；边界扇区或相电压顺序变化时断言失败
 */
static void check_sector_geometry(void)
{
    static const struct { float alpha, beta; uint32_t sector; } axes[] = {
        {0.0f, 0.0f, 1U}, {-0.0f, -0.0f, 1U},
        {0.25f, 0.0f, 1U}, {-0.25f, 0.0f, 3U},
        {0.0f, 0.25f, 2U}, {0.0f, -0.25f, 5U}
    };
    uint32_t a, b, c, sector;
    unsigned i;
    for (i = 0U; i < sizeof(axes) / sizeof(axes[0]); ++i) {
        foc_svm(axes[i].alpha, axes[i].beta, 5600U, &a, &b, &c, &sector);
        assert(sector == axes[i].sector);
    }
    /* 等于边界归前一扇区，向边界两侧各移动一个浮点单位。 */
    {
        float boundary = 0.57735026919f * 0.25f;
        foc_svm(boundary, 0.25f, 5600U, &a, &b, &c, &sector);
        assert(sector == 1U);
        foc_svm(nextafterf(boundary, -INFINITY), 0.25f, 5600U, &a, &b, &c, &sector);
        assert(sector == 2U);
        foc_svm(nextafterf(boundary, INFINITY), 0.25f, 5600U, &a, &b, &c, &sector);
        assert(sector == 1U);
    }
    for (i = 0U; i < 4096U; ++i) {
        const float turn = 6.283185307f;
        float angle = ((float)i + 0.375f) * turn / 4096.0f;
        float alpha = 0.25f * cosf(angle), beta = 0.25f * sinf(angle);
        float ta, tb, tc;
        uint32_t expected = (uint32_t)(angle / (turn / 6.0f)) + 1U;
        foc_svm(alpha, beta, 5600U, &a, &b, &c, &sector);
        assert(sector == expected);
        assert(svm(alpha, beta, &ta, &tb, &tc) == 0);
        assert(fabsf(ta + (float)a / 5600.0f - 1.0f) < 3.0f / 5600.0f);
        assert(fabsf(tb + (float)b / 5600.0f - 1.0f) < 3.0f / 5600.0f);
        assert(fabsf(tc + (float)c / 5600.0f - 1.0f) < 3.0f / 5600.0f);
    }
}

/**
 * @description: 独立三相平均电压方程核对真实调制实现，不使用输出函数桩。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
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
    check_sector_geometry();
    printf("linear SVM 16/24V, 3V sweep: max reconstruction error %.6fV; gain is 1, not 0.75\n", maximum);
    return 0;
}
