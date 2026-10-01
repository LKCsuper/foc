/*
 * @Description: 关节控制固定帧编解码；不操作SPI/CAN/UART硬件
 * @Author: lkc
 */
#ifndef MOTOR_JOINT_PROTOCOL_H
#define MOTOR_JOINT_PROTOCOL_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "motor_joint.h"
/* Private define ------------------------------------------------------------*/
#define MOTOR_JOINT_PROTOCOL_VERSION       (1U)
#define MOTOR_JOINT_COMMAND_FRAME_SIZE     (40U)
#define MOTOR_JOINT_STATUS_FRAME_SIZE      (40U)
/* Private typedef -----------------------------------------------------------*/
/**
 * @description: 协议层使用的电机力矩换算参数，不随每帧下发。
 */
typedef struct {
    float torque_constant_nm_per_a; /* 电机轴转矩常数 (N·m/A) */
    float gear_ratio;               /* 关节输出转矩 / 电机轴转矩，不得小于 1 */
    float efficiency;               /* 传动效率，范围 (0, 1] */
} motor_joint_protocol_motor_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 计算 CRC-32/ISO-HDLC。
 * @detail: poly=0x04C11DB7，init/xorout=0xFFFFFFFF，refin/refout=true。不访问硬件。
 * @param {const uint8_t *} data  待校验字节，为空时返回 0
 * @param {size_t} length  字节数
 * @return {uint32_t} 校验值
 * @author: lkc
 */
uint32_t Motor_JointProtocol_Crc32(const uint8_t *data, size_t length);

/**
 * @description: 解码 40 字节大端命令帧。
 * @detail: 帧头 FE EE。字段依次为 version、flags、seq、timeout_ms、position(µrad)、
 *          velocity(µrad/s)、torque_ff(µN·m)、kp、kd、torque_limit、current_limit(mA)、crc32。
 *          电机 Kt、减速比和效率来自 motor_config，不在帧内。
 * @param {const uint8_t[]} frame  40 字节命令帧
 * @param {const motor_joint_protocol_motor_t *} motor_config  力矩换算参数
 * @param {motor_joint_torque_command_t *} command  解码输出
 * @param {uint16_t *} sequence  帧序号输出
 * @return {bool} true=帧头、版本、保留位和 CRC 都通过
 * @author: lkc
 */
bool Motor_JointProtocol_DecodeCommand(const uint8_t frame[MOTOR_JOINT_COMMAND_FRAME_SIZE],
                                       const motor_joint_protocol_motor_t *motor_config,
                                       motor_joint_torque_command_t *command,
                                       uint16_t *sequence);

/**
 * @description: 把关节状态编成 40 字节大端状态帧。
 * @detail: 帧头 FC EE。位置用 µrad，速度用 µrad/s，转矩用 µN·m，电流和电压用 mA、mV。
 * @param {uint8_t[]} frame  40 字节输出缓冲
 * @param {uint16_t} sequence  回传序号
 * @param {const motor_joint_state_t *} state  当前关节状态
 * @param {uint32_t} fault_bits  故障位，原样写入帧内
 * @return {bool} true=已写完并附上 CRC；参数为空时返回 false
 * @author: lkc
 */
bool Motor_JointProtocol_EncodeStatus(uint8_t frame[MOTOR_JOINT_STATUS_FRAME_SIZE],
                                     uint16_t sequence,
                                     const motor_joint_state_t *state,
                                     uint32_t fault_bits);

#ifdef __cplusplus
}
#endif
#endif
