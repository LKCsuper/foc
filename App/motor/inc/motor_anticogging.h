/*
 * @Description: 编码器位置抗齿槽学习与 Vq 前馈表
 * @Author: lkc
 *
 * 336 桶覆盖一圈机械角，约每齿槽周期 4 个采样点。补偿电压为零均值，单位 V。
 */
#ifndef MOTOR_ANTICOGGING_H
#define MOTOR_ANTICOGGING_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
/* Private define ------------------------------------------------------------*/
/* 12 槽 14 极有 84 个齿槽周期；336 点提供每周期 4 个采样点。 */
#define MOTOR_ANTICOGGING_BINS (336U)
/* Private typedef -----------------------------------------------------------*/
/**
 * @description: 齿槽学习与补偿表的只读状态
 */
typedef struct {
    bool learning;           /* true=正在逐桶采样 */
    bool calibrated;         /* true=表已收成零均值补偿 */
    bool enabled;            /* true=运行中叠加补偿电压 */
    uint16_t target_samples; /* 每桶目标采样次数 */
    uint16_t min_samples;    /* 各桶已采样次数的最小值 */
    uint32_t total_samples;  /* 累计采样次数 */
    float min_voltage;       /* 表内最小补偿电压 (V) */
    float max_voltage;       /* 表内最大补偿电压 (V) */
} motor_anticogging_status_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 清空齿槽表并退出学习。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Anticogging_Init(void);

/**
 * @description: 开始一轮齿槽位置扫描。
 * @detail: 清空旧表并关闭补偿。每桶采样次数须在 1～5。
 * @param {uint16_t} samples_per_bin  每桶目标采样次数
 * @return {bool} true=已进入学习；false=次数非法
 * @author: lkc
 */
bool Motor_Anticogging_Start(uint16_t samples_per_bin);

/**
 * @description: 中止学习，不保留未完成的表。
 * @detail: 清学习标志和学习 Vq，不改已标定表的 calibrated/enabled。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Anticogging_Stop(void);

/**
 * @description: 清空补偿表、累加器和学习状态。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Anticogging_Clear(void);

/**
 * @description: 开关运行中的齿槽补偿。
 * @detail: 未标定完成时不允许打开。
 * @param {bool} enabled  true=叠加补偿电压
 * @return {bool} true=已设置；false=尚未标定却要求打开
 * @author: lkc
 */
bool Motor_Anticogging_SetEnabled(bool enabled);

/**
 * @description: 查询是否正在学习。
 * @return {bool} true=学习中，控制输出改由学习目标决定
 * @author: lkc
 */
bool Motor_Anticogging_IsLearning(void);

/**
 * @description: 写入本拍学习用的电角和保持电压。
 * @detail: 非学习中或参数非有限值时忽略。电角折回 [0, 2π)。
 * @param {float} encoder_elec_angle_rad  编码器电角 (rad)
 * @param {float} vq_target               维持位置的 q 轴电压 (V)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Anticogging_SetLearnTarget(float encoder_elec_angle_rad, float vq_target);

/**
 * @description: 读取学习拍要输出的电角和 dq 电压。
 * @detail: d 轴目标固定为 0。非学习中或指针为空时返回 false。
 * @param {float} *theta_rad  输出电角 (rad)
 * @param {float} *vd_target  输出 d 轴电压 (V)
 * @param {float} *vq_target  输出 q 轴电压 (V)
 * @return {bool} true=本拍由学习目标占用输出
 * @author: lkc
 */
bool Motor_Anticogging_GetLearnTargets(float *theta_rad, float *vd_target, float *vq_target);

/**
 * @description: 把一个桶的保持电压记入累加器。
 * @detail: 该桶达到目标次数后拒绝再记。不在学习中、桶号越界或电压非有限值时返回 false。
 * @param {uint16_t} bin         桶号，范围 [0, MOTOR_ANTICOGGING_BINS)
 * @param {float} holding_vq     该位置的保持电压 (V)
 * @return {bool} true=已记入
 * @author: lkc
 */
bool Motor_Anticogging_Record(uint16_t bin, float holding_vq);

/**
 * @description: 把各桶平均电压收成零均值补偿表并打开补偿。
 * @detail: 任一桶未达到目标次数则失败，表保持学习前状态。单点限幅 ±0.5 V。
 * @return {bool} true=已标定并使能
 * @author: lkc
 */
bool Motor_Anticogging_Finish(void);

/**
 * @description: 按机械角插值读取齿槽补偿电压。
 * @detail: 未使能、未标定或角度非有限值时返回 0。桶中心对齐，末桶与 0 桶衔接。
 * @param {float} mech_angle_rad  机械角 (rad)
 * @return {float} q 轴补偿电压 (V)
 * @author: lkc
 */
float Motor_Anticogging_GetVoltage(float mech_angle_rad);

/**
 * @description: 复制学习进度和补偿表电压范围。
 * @param {motor_anticogging_status_t} *status  输出，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Anticogging_GetStatus(motor_anticogging_status_t *status);

/**
 * @description: 导出已标定的补偿表和使能标志。
 * @detail: 未标定或仍在学习时失败，不写出表。
 * @param {float} *table     输出，长度须等于 MOTOR_ANTICOGGING_BINS，单位 V
 * @param {uint16_t} bins    调用方桶数，须等于 MOTOR_ANTICOGGING_BINS
 * @param {bool} *enabled    输出当前使能
 * @return {bool} true=已复制
 * @author: lkc
 */
bool Motor_Anticogging_Export(float *table, uint16_t bins, bool *enabled);

/**
 * @description: 从已校验的表恢复补偿，不重新学习。
 * @detail: 任一点非有限或超出 ±0.5 V 则整表拒绝，原状态不变。
 * @param {const float} *table  补偿电压 (V)，长度 bins
 * @param {uint16_t} bins       须等于 MOTOR_ANTICOGGING_BINS
 * @param {bool} enabled        恢复后是否使能
 * @return {bool} true=已标定并按 enabled 开关
 * @author: lkc
 */
bool Motor_Anticogging_Import(const float *table, uint16_t bins, bool enabled);
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
