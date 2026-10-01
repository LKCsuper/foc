/*
 * @Description: 关节控制固定帧编解码；全部多字节字段使用大端
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>
#include "motor_joint_protocol.h"

#define JOINT_CMD_HEAD0 (0xFEU)
#define JOINT_CMD_HEAD1 (0xEEU)
#define JOINT_STATE_HEAD0 (0xFCU)
#define JOINT_STATE_HEAD1 (0xEEU)
#define JOINT_FLAG_ENABLE (1U << 0)
#define JOINT_FLAG_TIMEOUT_ENABLE (1U << 1)
#define JOINT_RAD_PER_DEG (0.01745329251994329577f)
#define JOINT_RAD_S_PER_RPM (0.10471975511965977462f)

static uint16_t Joint_ReadU16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static uint32_t Joint_ReadU32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

static void Joint_WriteU16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value >> 8);
    p[1] = (uint8_t)value;
}

static void Joint_WriteU32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)(value >> 24);
    p[1] = (uint8_t)(value >> 16);
    p[2] = (uint8_t)(value >> 8);
    p[3] = (uint8_t)value;
}

static int32_t Joint_ScaleI32(float value, float scale)
{
    double scaled = (double)value * (double)scale;

    if (scaled > (double)INT32_MAX) return INT32_MAX;
    if (scaled < (double)INT32_MIN) return INT32_MIN;
    return (int32_t)(scaled >= 0.0 ? scaled + 0.5 : scaled - 0.5);
}

uint32_t Motor_JointProtocol_Crc32(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFFU;
    size_t i;

    if (data == NULL) return 0U;
    for (i = 0U; i < length; i++) {
        uint32_t value = data[i];
        unsigned int bit;

        crc ^= value;
        for (bit = 0U; bit < 8U; bit++) {
            crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320U : 0U);
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

bool Motor_JointProtocol_DecodeCommand(const uint8_t frame[MOTOR_JOINT_COMMAND_FRAME_SIZE],
                                       const motor_joint_protocol_motor_t *motor_config,
                                       motor_joint_torque_command_t *command,
                                       uint16_t *sequence)
{
    uint32_t expected_crc;
    uint32_t received_crc;
    uint16_t timeout_ms;
    uint8_t flags;

    if (frame == NULL || motor_config == NULL || command == NULL || sequence == NULL) return false;
    if (frame[0] != JOINT_CMD_HEAD0 || frame[1] != JOINT_CMD_HEAD1 ||
        frame[2] != MOTOR_JOINT_PROTOCOL_VERSION || (frame[3] & 0xFCU) != 0U) {
        return false;
    }
    expected_crc = Motor_JointProtocol_Crc32(frame, 36U);
    received_crc = Joint_ReadU32(&frame[36]);
    if (expected_crc != received_crc) return false;

    flags = frame[3];
    timeout_ms = Joint_ReadU16(&frame[6]);
    memset(command, 0, sizeof(*command));
    command->enable = (flags & JOINT_FLAG_ENABLE) != 0U;
    command->timeout_enable = (flags & JOINT_FLAG_TIMEOUT_ENABLE) != 0U;
    command->timeout_us = timeout_ms == 0U ? 0U : (uint32_t)timeout_ms * 1000U;
    command->position_rad = (float)(int32_t)Joint_ReadU32(&frame[8]) * 1.0e-6f;
    command->velocity_rad_s = (float)(int32_t)Joint_ReadU32(&frame[12]) * 1.0e-6f;
    command->torque_ff_nm = (float)(int32_t)Joint_ReadU32(&frame[16]) * 1.0e-6f;
    command->kp_nm_per_rad = (float)(int32_t)Joint_ReadU32(&frame[20]) * 1.0e-6f;
    command->kd_nm_per_rad_s = (float)(int32_t)Joint_ReadU32(&frame[24]) * 1.0e-6f;
    command->torque_limit_nm = (float)(int32_t)Joint_ReadU32(&frame[28]) * 1.0e-6f;
    command->current_limit_a = (float)(int16_t)Joint_ReadU16(&frame[32]) * 1.0e-3f;
    command->torque_constant_nm_per_a = motor_config->torque_constant_nm_per_a;
    command->gear_ratio = motor_config->gear_ratio;
    command->efficiency = motor_config->efficiency;
    *sequence = Joint_ReadU16(&frame[4]);
    return true;
}

bool Motor_JointProtocol_EncodeStatus(uint8_t frame[MOTOR_JOINT_STATUS_FRAME_SIZE],
                                     uint16_t sequence,
                                     const motor_joint_state_t *state,
                                     uint32_t fault_bits)
{
    uint8_t flags = 0U;
    uint32_t crc;
    uint32_t age_ms;

    if (frame == NULL || state == NULL) return false;
    memset(frame, 0, MOTOR_JOINT_STATUS_FRAME_SIZE);
    if (state->enabled) flags |= 1U << 0;
    if (state->command_timeout) flags |= 1U << 1;
    if (state->speed_valid) flags |= 1U << 2;
    if (state->encoder_fault) flags |= 1U << 3;
    if (state->backend != MOTOR_JOINT_BACKEND_VOLTAGE) flags |= 1U << 4;
    frame[0] = JOINT_STATE_HEAD0;
    frame[1] = JOINT_STATE_HEAD1;
    frame[2] = MOTOR_JOINT_PROTOCOL_VERSION;
    frame[3] = flags;
    Joint_WriteU16(&frame[4], sequence);
    age_ms = state->command_age_us / 1000U;
    Joint_WriteU16(&frame[6], age_ms > UINT16_MAX ? UINT16_MAX : (uint16_t)age_ms);
    Joint_WriteU32(&frame[8], (uint32_t)Joint_ScaleI32(state->position_deg * JOINT_RAD_PER_DEG, 1.0e6f));
    Joint_WriteU32(&frame[12], (uint32_t)Joint_ScaleI32(state->measured_rpm * JOINT_RAD_S_PER_RPM, 1.0e6f));
    Joint_WriteU32(&frame[16], (uint32_t)Joint_ScaleI32(state->torque_command_nm, 1.0e6f));
    Joint_WriteU32(&frame[20], (uint32_t)Joint_ScaleI32(state->iq_command_a, 1000.0f));
    Joint_WriteU32(&frame[24], (uint32_t)Joint_ScaleI32(state->output_v, 1000.0f));
    Joint_WriteU32(&frame[28], fault_bits);
    crc = Motor_JointProtocol_Crc32(frame, 36U);
    Joint_WriteU32(&frame[36], crc);
    return true;
}

#ifdef __cplusplus
}
#endif
