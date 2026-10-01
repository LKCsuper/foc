/*
 * @Description: 关节控制固定帧编解码；不操作SPI/CAN/UART硬件
 * @Author: lkc
 */
#ifndef MOTOR_JOINT_PROTOCOL_H
#define MOTOR_JOINT_PROTOCOL_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "motor_joint.h"

#define MOTOR_JOINT_PROTOCOL_VERSION       (1U)
#define MOTOR_JOINT_COMMAND_FRAME_SIZE     (40U)
#define MOTOR_JOINT_STATUS_FRAME_SIZE      (40U)

typedef struct {
    float torque_constant_nm_per_a;
    float gear_ratio;
    float efficiency;
} motor_joint_protocol_motor_t;

/** CRC-32/ISO-HDLC: poly=0x04C11DB7, init/xorout=0xFFFFFFFF, refin/refout=true。 */
uint32_t Motor_JointProtocol_Crc32(const uint8_t *data, size_t length);

/**
 * 命令帧40B，大端：FE EE/version/flags/seq/timeout_ms，随后依次为
 * position_urad、velocity_urad_s、torque_ff_uNm、kp_uNm_per_rad、
 * kd_uNm_per_rad_s、torque_limit_uNm、current_limit_mA、reserved、crc32。
 */
bool Motor_JointProtocol_DecodeCommand(const uint8_t frame[MOTOR_JOINT_COMMAND_FRAME_SIZE],
                                       const motor_joint_protocol_motor_t *motor_config,
                                       motor_joint_torque_command_t *command,
                                       uint16_t *sequence);

/** 状态帧40B，大端；编码当前关节状态、故障位、命令年龄和回传序号。 */
bool Motor_JointProtocol_EncodeStatus(uint8_t frame[MOTOR_JOINT_STATUS_FRAME_SIZE],
                                     uint16_t sequence,
                                     const motor_joint_state_t *state,
                                     uint32_t fault_bits);

#ifdef __cplusplus
}
#endif
#endif
