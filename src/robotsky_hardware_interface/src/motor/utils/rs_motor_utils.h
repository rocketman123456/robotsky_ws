#pragma once

#include "motor/motor_data.h"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>

#include <cstdint>

constexpr uint16_t RS_MASTER_ID = 0x40;

constexpr float RS_P_MIN  = -12.5f;
constexpr float RS_P_MAX  = 12.5f;
constexpr float RS_V_MIN  = -44.0f;
constexpr float RS_V_MAX  = 44.0f;
constexpr float RS_KP_MIN = 0.0f;
constexpr float RS_KP_MAX = 500.0f;
constexpr float RS_KD_MIN = 0.0f;
constexpr float RS_KD_MAX = 5.0f;
constexpr float RS_T_MIN  = -17.0f;
constexpr float RS_T_MAX  = 17.0f;

#define Set_mode 'j'      // 设置控制模式
#define Set_parameter 'p' // 设置参数
// 各种控制模式
#define move_control_mode 0  // 运控模式
#define Pos_control_mode 1   // 位置模式
#define Speed_control_mode 2 // 速度模式
#define Elect_control_mode 3 // 电流模式
#define Set_Zero_mode 4      // 零点模式
// 通信地址
constexpr uint32_t RS_Get_ID = 0x00;             // 获取设备的ID和64位MCU唯一标识符`
constexpr uint32_t RS_MotionControl = 0x01;      // 运控模式用来向主机发送控制指令
constexpr uint32_t RS_MotorRequest =0x02;       // 用来向主机反馈电机运行状态
constexpr uint32_t RS_MotorEnable =0x03;        // 电机使能运行
constexpr uint32_t RS_MotorStop =0x04;          // 电机停止运行
constexpr uint32_t RS_SetPosZero =0x06 ;        // 设置电机机械零位
constexpr uint32_t RS_Can_ID =0x07;             // 更改当前电机CAN_ID
constexpr uint32_t RS_Control_Mode =0x12;      // 设置电机模式
constexpr uint32_t RS_GetSingleParameter =0x11; // 读取单个参数
constexpr uint32_t RS_SetSingleParameter =0x12; // 设定单个参数
constexpr uint32_t RS_ErrorFeedback =0x15;      // 故障反馈帧

float rs_uint16_to_float(uint16_t x, float x_min, float x_max, int bits);
int   rs_float_to_uint(float x, float x_min, float x_max, int bits);
float rs_byte_to_float(uint8_t* bytedata);

void rs_enable_motor_mode(can_frame& frame, uint16_t motor_id);
void rs_disable_motor_mode(can_frame& frame, uint16_t motor_id, uint16_t clear_error);
