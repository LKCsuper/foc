/* 无 RTOS/板级依赖的统一调用约定断言。MCU 失败处理由 common.c 提供。 */
#ifndef COMMON_ASSERT_H
#define COMMON_ASSERT_H

#if !defined(__arm__) && !defined(__thumb__) && !defined(__CC_ARM) && !defined(__ICCARM__)
/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
/* 主机验证及独立算法示例：同样始终生效，失败终止进程。 */
/* Private macro -------------------------------------------------------------*/
#define COMMON_ASSERT(condition) do { if (!(condition)) { abort(); } } while (0)
#else
#ifdef __cplusplus
extern "C" {
#endif
/**
 * @description: 处理不可恢复的断言失败；关闭中断后停住，不返回。
 * @return {void} 无
 */
void Common_AssertFailed(void);
#ifdef __cplusplus
}
#endif
#define COMMON_ASSERT(condition) do { if (!(condition)) { Common_AssertFailed(); } } while (0)
#endif

#endif
