#pragma once

#include "motor/motor_data.h"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>

#include <cstdint>

constexpr uint16_t RS_MASTER_ID = 0x40;

static const uint16_t RS_Index_List[] = {
    0X7005, //
    0X7006,
    0X700A,
    0X700B,
    0X7010,
    0X7011,
    0X7014,
    0X7016,
    0X7017,
    0X7018,
    0x7019,
    0x701A,
    0x701B,
    0x701C,
    0x701D
};

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

constexpr uint8_t RS_Set_mode      = 'j'; // 设置控制模式
constexpr uint8_t RS_Set_parameter = 'p'; // 设置参数

// 各种控制模式
constexpr uint16_t RS_Move_Control_mode  = 0; // 运控模式
constexpr uint16_t RS_Pos_Control_mode   = 1; // 位置模式
constexpr uint16_t RS_Speed_Control_mode = 2; // 速度模式
constexpr uint16_t RS_Elect_Control_mode = 3; // 电流模式
constexpr uint16_t RS_Set_Zero_mode      = 4; // 零点模式

// 通信地址
constexpr uint32_t RS_Get_ID             = 0x00; // 获取设备的ID和64位MCU唯一标识符`
constexpr uint32_t RS_MotionControl      = 0x01; // 运控模式用来向主机发送控制指令
constexpr uint32_t RS_MotorRequest       = 0x02; // 用来向主机反馈电机运行状态
constexpr uint32_t RS_MotorEnable        = 0x03; // 电机使能运行
constexpr uint32_t RS_MotorStop          = 0x04; // 电机停止运行
constexpr uint32_t RS_SetPosZero         = 0x06; // 设置电机机械零位
constexpr uint32_t RS_Can_ID             = 0x07; // 更改当前电机CAN_ID
constexpr uint32_t RS_Control_Mode       = 0x12; // 设置电机模式
constexpr uint32_t RS_GetSingleParameter = 0x11; // 读取单个参数
constexpr uint32_t RS_SetSingleParameter = 0x12; // 设定单个参数
constexpr uint32_t RS_ErrorFeedback      = 0x15; // 故障反馈帧

class rs_data_read_write_one
{
public:
    uint16_t index;
    float    data;
};

class rs_data_read_write
{
public:
    rs_data_read_write_one run_mode;      // 0:运控模式 1:位置模式 2:速度模式 3:电流模式 4:零点模式 uint8  1byte
    rs_data_read_write_one iq_ref;        // 电流模式Iq指令  float 	4byte 	-23~23A
    rs_data_read_write_one spd_ref;       // 转速模式转速指令  float 	4byte 	-30~30rad/s
    rs_data_read_write_one limit_torque;  // 转矩限制  float 	4byte 	0~12Nm
    rs_data_read_write_one cur_kp;        // 电流的 Kp  float 	4byte 	默认值 0.125
    rs_data_read_write_one cur_ki;        // 电流的 Ki  float 	4byte 	默认值 0.0158
    rs_data_read_write_one cur_filt_gain; // 电流滤波系数filt_gain  float 	4byte 	0~1.0，默认值0.1
    rs_data_read_write_one loc_ref;       // 位置模式角度指令  float 	4byte 	rad
    rs_data_read_write_one limit_spd;     // 位置模式速度设置  float 	4byte 	0~30rad/s
    rs_data_read_write_one limit_cur;     // 速度位置模式电流设置  float 	4byte 	0~23A
    // 以下只可读
    rs_data_read_write_one mech_pos; // 负载端计圈机械角度  float 	4byte 	rad
    rs_data_read_write_one iqf;      // iq 滤波值  float 	4byte 	-23~23A
    rs_data_read_write_one mech_vel; // 负载端转速  float 	4byte 	-30~30rad/s
    rs_data_read_write_one vbus;     // 母线电压  float 	4byte 	V
    rs_data_read_write_one rotation; // 圈数  int16 	2byte   圈数

    rs_data_read_write(const uint16_t* index_list = RS_Index_List);
};

struct rs_motor_fb_t
{
    uint16_t id;
    int16_t  pattern; // 电机模式（0复位1标定2运行）
    float    pos;
    float    vel;
    float    tau;
    float    temp;
};

float rs_uint16_to_float(uint16_t x, float x_min, float x_max, int bits);
int   rs_float_to_uint(float x, float x_min, float x_max, int bits);
float rs_byte_to_float(uint8_t* bytedata);

void rs_enable_motor_mode(can_frame& frame, uint16_t motor_id);
void rs_disable_motor_mode(can_frame& frame, uint16_t motor_id, uint16_t clear_error);
void rs_set_motor_parameter(can_frame& frame, uint16_t motor_id, uint16_t index, float value, uint8_t value_mode);
void rs_get_motor_parameter(can_frame& frame, uint16_t motor_id, uint16_t index);
void rs_mit_ctrl(can_frame& frame, uint16_t motor_id, float pos, float vel, float kp, float kd, float tau);
void rs_pos_speed_ctrl(can_frame& frame, uint16_t motor_id, float pos, float vel);
void rs_zero_ctrl(can_frame& frame, uint16_t motor_id);
void rs_save_pos_zero(can_frame& frame, uint16_t motor_id);

void rs_decode(const can_frame& frame, rs_motor_fb_t& data, rs_data_read_write& drw);
