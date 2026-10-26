/* 霍尔标定纯状态机：不采集硬件，不等待，不自动应用结果。 */
#ifndef MOTOR_HALL_CALIBRATION_H
#define MOTOR_HALL_CALIBRATION_H
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Private typedef -----------------------------------------------------------*/
/* 标定状态机阶段：ALIGN..REVERSE 视为活动态，其余为终态/初始态。
 * DONE 后 table 才有效；CANCELLED/FAULT/TIMEOUT/INVALID 保留原表。 */
typedef enum {
    MOTOR_HALL_CAL_IDLE,     /* 初始态，未开始 */
    MOTOR_HALL_CAL_ALIGN,    /* 定角 Id 爬升对齐 */
    MOTOR_HALL_CAL_HOLD,     /* 对齐保持 */
    MOTOR_HALL_CAL_FORWARD,  /* +Iq 正向扫角采样 */
    MOTOR_HALL_CAL_REVERSE,  /* +Iq 反向扫角采样 */
    MOTOR_HALL_CAL_DONE,     /* 成功，已生成表 */
    MOTOR_HALL_CAL_CANCELLED,/* 被取消 */
    MOTOR_HALL_CAL_FAULT,    /* 保护故障退出 */
    MOTOR_HALL_CAL_TIMEOUT,  /* 超时退出 */
    MOTOR_HALL_CAL_INVALID   /* 参数或读数非法 */
} motor_hall_calibration_stage_t;

typedef struct {
    motor_hall_calibration_stage_t stage; /* 当前阶段 */
    uint32_t started_ms;                  /* 标定开始毫秒时间戳，超时基准 */
    uint32_t changed_ms;                  /* 上次阶段/步切换毫秒时间戳 */
    uint32_t timeout_ms;                  /* 总超时 (ms) */
    uint16_t step;                        /* 对齐爬升步数或扫角度步数 */
    uint8_t loop;                         /* 当前方向已扫圈数（0..LOOPS-1） */
    float current;                        /* 标定电流 (A) */
    float theta_rad;                      /* 当前强制电角度 (rad) */
    float id_target;                      /* D 轴目标电流 (A) */
    float iq_target;                      /* Q 轴目标电流 (A) */
    float sin_sum[8];                     /* 各霍尔状态 sin 累加 */
    float cos_sum[8];                     /* 各霍尔状态 cos 累加 */
    uint16_t samples[8];                  /* 各霍尔状态采样次数 */
    uint8_t table[8];                     /* 0..200 电角编码，255 无效；仅 DONE 后可读取结果 */
    bool sampled_forward;                 /* 本次 Step 是否接受了正向样本，供应用记录换相序列 */
} motor_hall_calibration_t;

/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 开始/重新开始标定。state 必需非空；current 为有限正电流 (A)。
 * now_ms 及后续 Step 使用同一 uint32 毫秒时基，支持一次回绕。
 * timeout_ms 须在 1..INT32_MAX；失败返回 false 并置 INVALID、清零目标。
 * 调用者先停机，再独占输出；不能对正在由另一上下文步进的实例调用。
 * @param {motor_hall_calibration_t *} state  标定状态，不可为空
 * @param {float} current                     标定电流 (A)
 * @param {uint32_t} now_ms                   开始时刻 (ms)
 * @param {uint32_t} timeout_ms               总超时 (ms)
 * @return {bool} true=已进入 ALIGN 阶段；false=参数非法
 */
bool Motor_Hall_CalibrationStart(motor_hall_calibration_t *state, float current, uint32_t now_ms,
                                 uint32_t timeout_ms);
/**
 * @description: 非阻塞推进一步；hall 为本次调用采集的 0..7 原始读数，仅扫角到期时消费。
 * fault 优先于 cancel，均优先于超时/推进；异常读数在消费时以 INVALID 退出。
 * 终态幂等且电流/角度目标清零。调用者必须在终态撤销标定输出权并停机。
 * 本接口不应用霍尔表、不清故障、不恢复运行命令。必需指针非空。
 * @param {motor_hall_calibration_t *} state  标定状态，不可为空
 * @param {uint32_t} now_ms                   本次时刻 (ms)
 * @param {int} hall                          本次读取的霍尔状态 0..7
 * @param {bool} cancel                       true=请求取消
 * @param {bool} fault                        true=发生保护故障
 * @return {void} 无
 */
void Motor_Hall_CalibrationStep(motor_hall_calibration_t *state, uint32_t now_ms, int hall,
                                bool cancel, bool fault);
/**
 * @description: 查询是否仍占用标定输出；state 必需非空。
 * @param {const motor_hall_calibration_t *} state  标定状态，不可为空
 * @return {bool} true=处于 ALIGN..REVERSE 活动态
 */
bool Motor_Hall_CalibrationIsActive(const motor_hall_calibration_t *state);

#ifdef __cplusplus
}
#endif
#endif
