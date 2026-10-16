#ifndef MOTOR_TASK_CLOCK_H
#define MOTOR_TASK_CLOCK_H
#include <stdint.h>

/** 返回下一未来节拍，跳过已错过的拍，不追赶补算；周期和滞后须小于2^31 tick。 */
static inline uint32_t Motor_Task_NextDeadline(uint32_t previous, uint32_t now, uint32_t period)
{
    uint32_t next;
    if (period == 0U) period = 1U;
    next = previous + period;
    if ((int32_t)(now - next) >= 0) {
        next += ((now - next) / period + 1U) * period;
    }
    return next;
}
#endif
