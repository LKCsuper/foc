/*
 * @Description: UVW 线序检测（开环正转电角 vs 霍尔扇区步进方向）
 * @Author: lkc
 *
 * 学习要点：
 *   1) 无编码器时用三相霍尔：软件正转下扇区应按固定六步序前进；反向则交换 U/V。
 *   2) 不依赖已标定 hall_table；只依赖 1..6 有效状态与标准相邻关系。
 *   3) Judge / HallStep 无硬件可主机单测；Detect 为板端阻塞 V/F 慢转。
 */
#ifndef MOTOR_PHASE_ORDER_H
#define MOTOR_PHASE_ORDER_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/**
 * @description: 线序检测结论
 */
typedef enum {
    MOTOR_PHASE_ORDER_OK = 0,     /* 线序与软件正方向一致 */
    MOTOR_PHASE_ORDER_SWAP_UV,    /* 霍尔步进反向，建议交换 U/V */
    MOTOR_PHASE_ORDER_NO_MOTION,  /* 霍尔几乎无扇区变化，拖不动或未接 */
    MOTOR_PHASE_ORDER_FAULT,      /* 过程中保护锁存 */
    MOTOR_PHASE_ORDER_BUSY,       /* 电机正在跑或其它校准占用 */
    MOTOR_PHASE_ORDER_INVALID,    /* 参数非法 */
} motor_phase_order_result_t;

/**
 * @description: 一次检测的可读报告
 */
typedef struct {
    motor_phase_order_result_t result;
    int32_t dir_delta;     /* 霍尔扇区步进累加：正=标准正序，负=反序 */
    float commanded_rpm;   /* 本次开环目标机械转速 (RPM) */
    float duration_s;      /* 实际转时 (s) */
    float boost_v;         /* V/F 使用的 boost (V) */
} motor_phase_order_report_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 相邻两次霍尔状态的扇区步进（标准六步序）
 * @param {int} prev_hall  上一霍尔 0..7
 * @param {int} curr_hall  当前霍尔 0..7
 * @return {int32_t} +1 正序一步，-1 反序一步，0 无效/同态/跳步
 * @author: lkc
 */
int32_t Motor_PhaseOrder_HallStep(int prev_hall, int curr_hall);

/**
 * @description: 由方向得分与命令转速符号判定线序
 * @param {int32_t} dir_delta       有符号方向得分（霍尔步进累加）
 * @param {float} commanded_rpm     开环目标转速 (RPM)，正=软件正转
 * @param {int32_t} min_steps       |delta| 小于此值视为无有效运动
 * @return {motor_phase_order_result_t} OK / SWAP_UV / NO_MOTION / INVALID
 * @author: lkc
 */
motor_phase_order_result_t Motor_PhaseOrder_Judge(int32_t dir_delta, float commanded_rpm, int32_t min_steps);

/**
 * @description: 结论转短字符串（静态字面量）
 * @param {motor_phase_order_result_t} result  结论
 * @return {const char *} 英文短名
 * @author: lkc
 */
const char *Motor_PhaseOrder_ResultName(motor_phase_order_result_t result);

/**
 * @description: 板上执行 UVW 线序检测（V/F 正转慢爬 + 读霍尔扇区）
 * @detail: 任务上下文阻塞；会停机、切开环 V/F、短时使能后停止。
 *          轴须可自由转；霍尔须能读到 1..6；建议先 olfault clear。
 * @param {motor_phase_order_report_t} *report  输出报告，不可为空
 * @return {motor_phase_order_result_t} 与 report->result 相同
 * @author: lkc
 */
motor_phase_order_result_t Motor_PhaseOrder_Detect(motor_phase_order_report_t *report);

#ifdef __cplusplus
}
#endif
#endif
