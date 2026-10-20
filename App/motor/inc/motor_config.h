/*
 * @Description: 电机框架编译期模块选择
 * @Author: lkc
 * @detail: 每个开关默认 1（BM75 全功能）。移植工程可用编译器 -D 覆盖，或定义
 *          MOTOR_CONFIG_USER_FILE 指向工程自己的配置头（如 -DMOTOR_CONFIG_USER_FILE="\"foc_user_config.h\""）。
 *          关闭的模块整个 .c 编译为空，头文件给出内联空实现，调用方无需改动；工程文件列表保持不变。
 */
#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#ifdef MOTOR_CONFIG_USER_FILE
#include MOTOR_CONFIG_USER_FILE
#endif
/* Private define ------------------------------------------------------------*/
/* 调度/轨迹/交接/半周期配对等运行诊断及 cmd_motor_diag 命令；关闭后约省 52 KB RAM（CCM）。 */
#ifndef MOTOR_CFG_DIAG
#define MOTOR_CFG_DIAG 1
#endif

/* BLDC 六步换相模式（含其 ADC/TIM/DMA 切换）；关闭后 ISR 只走 FOC。 */
#ifndef MOTOR_CFG_BLDC
#define MOTOR_CFG_BLDC 1
#endif

/* 关节模式（单编码器位置/阻抗控制与协议）。 */
#ifndef MOTOR_CFG_JOINT
#define MOTOR_CFG_JOINT 1
#endif

/* 电机参数辨识：R/L/磁链检测与电流环 PI 自整定。电流零点校准不属于本开关，始终保留。 */
#ifndef MOTOR_CFG_IDENTIFY
#define MOTOR_CFG_IDENTIFY 1
#endif

/* 串口命令行；关闭后只保留 C 接口（Motor_RequestStart/Stop 等），由上层应用调用。 */
#ifndef MOTOR_CFG_CMD
#define MOTOR_CFG_CMD 1
#endif

/* 模块间依赖：命令只随其模块编入，模块本身不依赖命令行。 */
#define MOTOR_CFG_CMD_DIAG (MOTOR_CFG_CMD && MOTOR_CFG_DIAG)
#define MOTOR_CFG_CMD_BLDC (MOTOR_CFG_CMD && MOTOR_CFG_BLDC)
#define MOTOR_CFG_CMD_JOINT (MOTOR_CFG_CMD && MOTOR_CFG_JOINT)
#define MOTOR_CFG_CMD_IDENTIFY (MOTOR_CFG_CMD && MOTOR_CFG_IDENTIFY)

#if (MOTOR_CFG_DIAG != 0 && MOTOR_CFG_DIAG != 1) || (MOTOR_CFG_BLDC != 0 && MOTOR_CFG_BLDC != 1) ||                \
    (MOTOR_CFG_JOINT != 0 && MOTOR_CFG_JOINT != 1) || (MOTOR_CFG_IDENTIFY != 0 && MOTOR_CFG_IDENTIFY != 1) ||       \
    (MOTOR_CFG_CMD != 0 && MOTOR_CFG_CMD != 1)
#error "MOTOR_CFG_* must be 0 or 1"
#endif
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
