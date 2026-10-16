/* 主机绕组模型：生产 PI、坐标变换和 SVPWM，模型参数仅用于测试。 */
#include <math.h>
#include <stdio.h>
#include "motor_pid.h"
#include "motor_foc.h"

static int checks;
static int failures;
#define CHECK(condition) do { checks++; if (!(condition)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); failures++; } } while (0)

static void run_case(float resistance, float inductance, float dt, int quantized)
{
    float current = 0.0f, integral = 0.0f;
    float max_error = 0.0f;
    const float bus = 24.0f, limit = 12.0f;
    int count = (int)(0.4f / dt);
    for (int k = 0; k < count; k++) {
        float time = k * dt;
        float target = time < 0.1f ? 1.0f : (time < 0.2f ? -1.0f : 0.0f);
        float measured = quantized ? roundf(current / 0.0806f) * 0.0806f : current;
        float voltage = Motor_Current_PI_Run(target, measured, &integral,
                                              inductance * 1000.0f, resistance * 1000.0f, dt, limit);
        float theta = time * 90.0f;
        float alpha, beta, va, vb, vc, id, iq;
        uint32_t a, b, c, sector;
        inverse_park(voltage, 0.0f, theta, &alpha, &beta);
        foc_svm_from_voltage(alpha, beta, bus, 0.95f, 4000U, &a, &b, &c, &sector);
        CHECK(a <= 4000U && b <= 4000U && c <= 4000U);
        /* 用实际比较值还原平均相电压，包含 PWM 计数量化。 */
        va = bus * a / 4000.0f;
        vb = bus * b / 4000.0f;
        vc = bus * c / 4000.0f;
        clarke_transform(va, vb, vc, &alpha, &beta);
        park_transform(alpha, beta, theta, &id, &iq);
        /* 锁轴等效 RL 模型的精确离散解；不模拟机械运动和功率开关瞬态。 */
        float decay = expf(-resistance * dt / inductance);
        /* 生产 inverse_park 保留板级负极性；模型按同一电流正方向还原。 */
        current = current * decay - id / resistance * (1.0f - decay);
        CHECK(isfinite(current) && isfinite(integral));
        CHECK(fabsf(voltage) <= limit + 1e-5f);
        if ((time > 0.07f && time < 0.1f) || (time > 0.17f && time < 0.2f) || time > 0.27f) {
            float error = fabsf(current - target);
            if (error > max_error) max_error = error;
        }
    }
    printf("RL R=%.3f L=%.6f dt=%.6f ADC=%s settled_error=%.6f A\n",
           resistance, inductance, dt, quantized ? "quantized" : "ideal", max_error);
    CHECK(max_error < (quantized ? 0.10f : 0.02f));
}

static void saturation_recovery(void)
{
    float current = 0.0f, integral = 0.0f;
    const float dt = 0.00005f;
    for (int k = 0; k < 8000; k++) {
        float target = k < 2000 ? 100.0f : 0.0f;
        float v = Motor_Current_PI_Run(target, current, &integral, 1.0f, 500.0f, dt, 2.0f);
        current = current * expf(-dt / 0.002f) + v * (1.0f - expf(-dt / 0.002f));
        CHECK(isfinite(integral) && fabsf(v) <= 2.00001f);
        if (k == 1999) CHECK(current > 1.9f && current < 2.01f);
    }
    CHECK(fabsf(current) < 0.01f);
    CHECK(fabsf(integral) < 0.01f);
    printf("Unreachable target -> zero: current=%.6f integral=%.6f\n", current, integral);
}

int main(void)
{
    float resistances[] = {0.5f, 3.75f};
    float inductances[] = {0.001f, 0.003f};
    float periods[] = {0.00005f, 0.0001f};
    for (int r = 0; r < 2; r++)
        for (int l = 0; l < 2; l++)
            for (int d = 0; d < 2; d++)
                for (int q = 0; q < 2; q++)
                    run_case(resistances[r], inductances[l], periods[d], q);
    saturation_recovery();
    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
