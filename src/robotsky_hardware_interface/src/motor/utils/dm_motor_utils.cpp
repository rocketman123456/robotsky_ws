#include "motor/utils/dm_motor_utils.h"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>

#include <cstdint>

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int dm_float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    /* Converts a float to an unsigned int, given range and number of bits */
    float span   = x_max - x_min;
    float offset = x_min;
    return static_cast<int>((x_float - offset) * (static_cast<float>((1 << bits) - 1)) / span);
}

/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float dm_uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /* converts unsigned int to float, given range and number of bits */
    float span   = x_max - x_min;
    float offset = x_min;
    return (static_cast<float>(x_int)) * span / (static_cast<float>((1 << bits) - 1)) + offset;
}

/**
************************************************************************
* @brief:      	enable_motor_mode: 启用电机模式函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要开启的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送启用特定模式的命令
************************************************************************
**/
void dm_enable_motor_mode(can_frame& frame, uint16_t motor_id, uint16_t mode_id)
{
    frame.can_id  = motor_id + mode_id;
    frame.can_dlc = 8;

    frame.data[0] = 0xFF;
    frame.data[1] = 0xFF;
    frame.data[2] = 0xFF;
    frame.data[3] = 0xFF;
    frame.data[4] = 0xFF;
    frame.data[5] = 0xFF;
    frame.data[6] = 0xFF;
    frame.data[7] = 0xFC;
}

/**
************************************************************************
* @brief:      	disable_motor_mode: 禁用电机模式函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要禁用的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送禁用特定模式的命令
************************************************************************
**/
void dm_disable_motor_mode(can_frame& frame, uint16_t motor_id, uint16_t mode_id)
{
    frame.can_id  = motor_id + mode_id;
    frame.can_dlc = 8;

    frame.data[0] = 0xFF;
    frame.data[1] = 0xFF;
    frame.data[2] = 0xFF;
    frame.data[3] = 0xFF;
    frame.data[4] = 0xFF;
    frame.data[5] = 0xFF;
    frame.data[6] = 0xFF;
    frame.data[7] = 0xFD;
}

void dm_mit_ctrl(can_frame& frame, uint16_t motor_id, float pos, float vel, float kp, float kd, float tau)
{
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

    pos_tmp = dm_float_to_uint(pos, DM_P_MIN, DM_P_MAX, 16);
    vel_tmp = dm_float_to_uint(vel, DM_V_MIN, DM_V_MAX, 12);
    kp_tmp  = dm_float_to_uint(kp, DM_KP_MIN, DM_KP_MAX, 12);
    kd_tmp  = dm_float_to_uint(kd, DM_KD_MIN, DM_KD_MAX, 12);
    tor_tmp = dm_float_to_uint(tau, DM_T_MIN, DM_T_MAX, 12);

    frame.can_id  = motor_id + DM_MIT_MODE;
    frame.can_dlc = 8;

    frame.data[0] = (pos_tmp >> 8);
    frame.data[1] = pos_tmp;
    frame.data[2] = (vel_tmp >> 4);
    frame.data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    frame.data[4] = kp_tmp;
    frame.data[5] = (kd_tmp >> 4);
    frame.data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    frame.data[7] = tor_tmp;
}

/**
************************************************************************
* @brief:      	speed_ctrl: 速度控制函数
* @param[in]:   hcan: 		指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   vel: 			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送速度控制命令
************************************************************************
**/
void dm_pos_speed_ctrl(can_frame& frame, uint16_t motor_id, float pos, float vel)
{
    uint8_t *pbuf, *vbuf;

    frame.can_id  = motor_id + DM_POS_MODE;
    frame.can_dlc = 8;

    pbuf = reinterpret_cast<uint8_t*>(&pos);
    vbuf = reinterpret_cast<uint8_t*>(&vel);

    frame.data[0] = *pbuf;
    frame.data[1] = *(pbuf + 1);
    frame.data[2] = *(pbuf + 2);
    frame.data[3] = *(pbuf + 3);

    frame.data[4] = *vbuf;
    frame.data[5] = *(vbuf + 1);
    frame.data[6] = *(vbuf + 2);
    frame.data[7] = *(vbuf + 3);
}

/**
************************************************************************
* @brief:      	pos_speed_ctrl: 位置速度控制函数
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   vel:			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送位置速度控制命令
************************************************************************
**/
void dm_speed_ctrl(can_frame& frame, uint16_t motor_id, float vel)
{
    uint8_t* vbuf;

    frame.can_id  = motor_id + DM_SPEED_MODE;
    frame.can_dlc = 4;

    vbuf = (uint8_t*)&vel;

    frame.data[0] = *vbuf;
    frame.data[1] = *(vbuf + 1);
    frame.data[2] = *(vbuf + 2);
    frame.data[3] = *(vbuf + 3);
}

/**
************************************************************************
* @brief:      	save_pos_zero: 保存位置零点函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要保存位置零点的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送保存位置零点的命令
************************************************************************
**/
void dm_save_pos_zero(can_frame& frame, uint16_t motor_id, uint16_t mode_id)
{
    frame.can_id  = motor_id + mode_id;
    frame.can_dlc = 8;

    frame.data[0] = 0xFF;
    frame.data[1] = 0xFF;
    frame.data[2] = 0xFF;
    frame.data[3] = 0xFF;
    frame.data[4] = 0xFF;
    frame.data[5] = 0xFF;
    frame.data[6] = 0xFF;
    frame.data[7] = 0xFE;
}

/**
************************************************************************
* @brief:      	clear_err: 清除电机错误函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要清除错误的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送清除错误的命令。
************************************************************************
**/
void dm_clear_err(can_frame& frame, uint16_t motor_id, uint16_t mode_id)
{
    frame.can_id  = motor_id + mode_id;
    frame.can_dlc = 8;

    frame.data[0] = 0xFF;
    frame.data[1] = 0xFF;
    frame.data[2] = 0xFF;
    frame.data[3] = 0xFF;
    frame.data[4] = 0xFF;
    frame.data[5] = 0xFF;
    frame.data[6] = 0xFF;
    frame.data[7] = 0xFB;
}

/**
************************************************************************
* @brief:      	dm4310_fbdata: 获取DM4310电机反馈数据函数
* @param[in]:   motor:    指向motor_t结构的指针，包含电机相关信息和反馈数据
* @param[in]:   rx_data:  指向包含反馈数据的数组指针
* @retval:     	void
* @details:    	从接收到的数据中提取DM4310电机的反馈信息，包括电机ID、
*               状态、位置、速度、扭矩以及相关温度参数
************************************************************************
**/
void dm_decode(const can_frame& frame, dm_motor_fb_t& data)
{
    // frame.can_id  = motor_id + mode_id;
    // frame.can_dlc = 8;

    data.id    = (frame.data[0]) & 0x0F;
    data.state = (frame.data[0]) >> 4;
    data.p_int = (frame.data[1] << 8) | frame.data[2];
    data.v_int = (frame.data[3] << 4) | (frame.data[4] >> 4);
    data.t_int = ((frame.data[4] & 0xF) << 8) | frame.data[5];
    data.pos   = dm_uint_to_float(data.p_int, DM_P_MIN, DM_P_MAX, 16); // (-12.5,12.5)
    data.vel   = dm_uint_to_float(data.v_int, DM_V_MIN, DM_V_MAX, 12); // (-45.0,45.0)
    data.tor   = dm_uint_to_float(data.t_int, DM_T_MIN, DM_T_MAX, 12); // (-18.0,18.0)
    data.t_mos  = (float)(frame.data[6]);
    data.t_coil = (float)(frame.data[7]);
}
