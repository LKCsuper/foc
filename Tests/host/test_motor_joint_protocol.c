#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "motor_joint_protocol.h"

static void write_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value >> 8);
    p[1] = (uint8_t)value;
}

static void write_u32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)(value >> 24);
    p[1] = (uint8_t)(value >> 16);
    p[2] = (uint8_t)(value >> 8);
    p[3] = (uint8_t)value;
}

int main(void)
{
    static const uint8_t check[] = "123456789";
    uint8_t frame[MOTOR_JOINT_COMMAND_FRAME_SIZE] = {0};
    uint8_t status_frame[MOTOR_JOINT_STATUS_FRAME_SIZE];
    motor_joint_protocol_motor_t motor_config = {0.08f, 9.0f, 0.9f};
    motor_joint_torque_command_t command;
    motor_joint_state_t state;
    uint16_t sequence;

    assert(Motor_JointProtocol_Crc32(check, 9U) == 0xCBF43926U);
    frame[0] = 0xFEU;
    frame[1] = 0xEEU;
    frame[2] = MOTOR_JOINT_PROTOCOL_VERSION;
    frame[3] = 0x03U;
    write_u16(&frame[4], 0x1234U);
    write_u16(&frame[6], 100U);
    write_u32(&frame[8], 1000000U);
    write_u32(&frame[12], (uint32_t)-2000000);
    write_u32(&frame[16], 100000U);
    write_u32(&frame[20], 2000000U);
    write_u32(&frame[24], 300000U);
    write_u32(&frame[28], 500000U);
    write_u16(&frame[32], 1500U);
    write_u32(&frame[36], Motor_JointProtocol_Crc32(frame, 36U));
    assert(Motor_JointProtocol_DecodeCommand(frame, &motor_config, &command, &sequence));
    assert(sequence == 0x1234U);
    assert(command.enable && command.timeout_enable && command.timeout_us == 100000U);
    assert(fabsf(command.position_rad - 1.0f) < 1e-6f);
    assert(fabsf(command.velocity_rad_s + 2.0f) < 1e-6f);
    assert(fabsf(command.current_limit_a - 1.5f) < 1e-6f);
    frame[10] ^= 1U;
    assert(!Motor_JointProtocol_DecodeCommand(frame, &motor_config, &command, &sequence));

    memset(&state, 0, sizeof(state));
    state.enabled = true;
    state.speed_valid = true;
    state.backend = MOTOR_JOINT_BACKEND_TORQUE_CURRENT;
    state.position_deg = 57.2957795f;
    state.measured_rpm = 9.5492966f;
    state.torque_command_nm = 0.1f;
    state.iq_command_a = 1.25f;
    state.command_age_us = 123000U;
    assert(Motor_JointProtocol_EncodeStatus(status_frame, 7U, &state, 0x12345678U));
    assert(status_frame[0] == 0xFCU && status_frame[1] == 0xEEU);
    assert(Motor_JointProtocol_Crc32(status_frame, 36U) ==
           ((uint32_t)status_frame[36] << 24 | (uint32_t)status_frame[37] << 16 |
            (uint32_t)status_frame[38] << 8 | status_frame[39]));

    puts("test_motor_joint_protocol: pass");
    return 0;
}
