/* host 测试用最小 FreeRTOS stub：仅提供 motor_hall.c 任务包装依赖的毫秒→tick 换算。 */
#ifndef HALL_TASK_TEST_FREERTOS_H
#define HALL_TASK_TEST_FREERTOS_H
/* Private macro -------------------------------------------------------------*/
#define pdMS_TO_TICKS(ms) (ms)
#endif
