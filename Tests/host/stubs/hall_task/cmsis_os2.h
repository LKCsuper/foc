/* host 测试用最小 CMSIS-RTOS2 stub：仅声明 osDelay，由测试文件提供实现。 */
#ifndef HALL_TASK_TEST_CMSIS_OS2_H
#define HALL_TASK_TEST_CMSIS_OS2_H
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/**
 * @description: 主机测试替身：模拟任务延时，按本测试场景推进时间或注入事件。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {uint32_t} ticks ticks 参数
 * @return {int32_t} 本次计算或查询的结果
 */
int32_t osDelay(uint32_t ticks);
#endif
