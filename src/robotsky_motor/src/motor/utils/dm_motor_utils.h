#pragma once

#include "motor/motor_data.h"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>

#include <algorithm>
#include <cstdint>

constexpr uint16_t DM_MIT_MODE   = 0x000;
constexpr uint16_t DM_POS_MODE   = 0x100;
constexpr uint16_t DM_SPEED_MODE = 0x200;

constexpr uint16_t DM_MASTER_ID = 0x40;

constexpr float DM_P_MIN  = -12.5f;
constexpr float DM_P_MAX  = 12.5f;
constexpr float DM_V_MIN  = -30.0f;
constexpr float DM_V_MAX  = 30.0f;
constexpr float DM_KP_MIN = 0.0f;
constexpr float DM_KP_MAX = 500.0f;
constexpr float DM_KD_MIN = 0.0f;
constexpr float DM_KD_MAX = 5.0f;
constexpr float DM_T_MIN  = -10.0f;
constexpr float DM_T_MAX  = 10.0f;

// 电机回传信息结构体
struct dm_motor_fb_t
{
    int   mst_id = 0;
    int   id = 0;
    int   state = 0;
    int   p_int = 0;
    int   v_int = 0;
    int   t_int = 0;
    int   kp_int = 0;
    int   kd_int = 0;
    float pos = 0;
    float vel = 0;
    float tau = 0;
    float kp = 0;
    float kd = 0;
    float t_mos = 0;
    float t_coil = 0;
};

// 电机参数设置结构体
struct dm_motor_ctrl_t
{
    int8_t mode;
    float  pos_set;
    float  vel_set;
    float  tor_set;
    float  kp_set;
    float  kd_set;
};

struct dm_motor_data_t
{
    int8_t          id;
    uint8_t         start_flag;
    dm_motor_fb_t   para;
    dm_motor_ctrl_t ctrl;
    dm_motor_ctrl_t cmd;
};

float dm_uint_to_float(int x_int, float x_min, float x_max, int bits);
int   dm_float_to_uint(float x_float, float x_min, float x_max, int bits);

void dm_enable_motor_mode(can_frame& frame, uint16_t motor_id, uint16_t mode_id);
void dm_disable_motor_mode(can_frame& frame, uint16_t motor_id, uint16_t mode_id);
void dm_mit_ctrl(can_frame& frame, uint16_t motor_id, float pos, float vel, float kp, float kd, float tau);
void dm_pos_speed_ctrl(can_frame& frame, uint16_t motor_id, float pos, float vel);
void dm_speed_ctrl(can_frame& frame, uint16_t motor_id, float vel);
void dm_save_pos_zero(can_frame& frame, uint16_t motor_id, uint16_t mode_id);
void dm_clear_err(can_frame& frame, uint16_t motor_id, uint16_t mode_id);

void dm_decode(const can_frame& frame, dm_motor_fb_t& data);
