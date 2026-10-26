/*
 * @Description: 周期任务截止时间计算；跳过错过的节拍，不追赶补算。
 */
#ifndef MOTOR_TASK_CLOCK_H
#define MOTOR_TASK_CLOCK_H
#include <stdint.h>
#include <stddef.h>
#include "common_assert.h"

typedef struct {
    uint32_t tick_hz, period;
    float loop_hz;
} motor_task_period_cache_t;

/**
 * @description: 配置变化时换算任务周期，其余调用复用缓存。
 * @detail: cache不可为空，首次使用须清零；Hz转tick保持截断且至少1tick，低于1Hz或NaN按1Hz处理。
 * @param {motor_task_period_cache_t *} cache 任务私有缓存
 * @param {uint32_t} tick_hz 内核tick频率，0按1Hz
 * @param {float} loop_hz 外环频率，单位Hz
 * @return {uint32_t} 周期tick，不改变截止时间策略
 */
static inline uint32_t Motor_Task_PeriodTicks(motor_task_period_cache_t *cache, uint32_t tick_hz, float loop_hz)
{
    float ticks;
    COMMON_ASSERT(cache != NULL);
    if (tick_hz == 0U) tick_hz = 1U;
    if (!(loop_hz >= 1.0f)) loop_hz = 1.0f;
    if (cache->period && cache->tick_hz == tick_hz && cache->loop_hz == loop_hz) return cache->period;
    ticks = (float)tick_hz / loop_hz;
    cache->period = ticks >= 4294967296.0f ? UINT32_MAX : (uint32_t)ticks;
    if (cache->period == 0U) cache->period = 1U;
    cache->tick_hz = tick_hz;
    cache->loop_hz = loop_hz;
    return cache->period;
}

/**
 * @description: 返回下一未来节拍，跳过已错过的拍，不追赶补算；周期和滞后须小于2^31 tick。
 * @param {uint32_t} previous 上一次计划截止时刻 (tick)
 * @param {uint32_t} now 当前时刻 (tick)
 * @param {uint32_t} period 调度周期 (tick)，须非零且小于 2^31
 * @return {uint32_t} 当前值 next
 */
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
