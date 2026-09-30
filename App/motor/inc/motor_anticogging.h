/* @Description: 编码器位置抗齿槽学习与 Vq 前馈表 */
#ifndef MOTOR_ANTICOGGING_H
#define MOTOR_ANTICOGGING_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdbool.h>
#include <stdint.h>

/* 12槽14极有84个齿槽周期；336点提供每周期4个采样点。 */
#define MOTOR_ANTICOGGING_BINS (336U)

typedef struct {
    bool learning;
    bool calibrated;
    bool enabled;
    uint16_t target_samples;
    uint16_t min_samples;
    uint32_t total_samples;
    float min_voltage;
    float max_voltage;
} motor_anticogging_status_t;

void Motor_Anticogging_Init(void);
bool Motor_Anticogging_Start(uint16_t samples_per_bin);
void Motor_Anticogging_Stop(void);
void Motor_Anticogging_Clear(void);
bool Motor_Anticogging_SetEnabled(bool enabled);
bool Motor_Anticogging_IsLearning(void);
void Motor_Anticogging_SetLearnTarget(float encoder_elec_angle_rad, float vq_target);
bool Motor_Anticogging_GetLearnTargets(float *theta_rad, float *vd_target, float *vq_target);
bool Motor_Anticogging_Record(uint16_t bin, float holding_vq);
bool Motor_Anticogging_Finish(void);
float Motor_Anticogging_GetVoltage(float mech_angle_rad);
void Motor_Anticogging_GetStatus(motor_anticogging_status_t *status);
bool Motor_Anticogging_Export(float *table, uint16_t bins, bool *enabled);
bool Motor_Anticogging_Import(const float *table, uint16_t bins, bool enabled);

#ifdef __cplusplus
}
#endif
#endif
