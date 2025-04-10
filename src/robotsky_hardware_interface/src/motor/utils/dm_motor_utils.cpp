#include "motor/utils/dm_motor_utils.h"

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

void dm_enable_motor_mode(can_frame& frame, uint16_t motor_id, uint16_t mode_id)
{
    //
}

void dm_disable_motor_mode(can_frame& frame, uint16_t motor_id)
{
    //
}

void dm_mit_ctrl(can_frame& frame, uint16_t motor_id, float pos, float vel, float kp, float kd, float tau)
{
    //
}

void dm_pos_speed_ctrl(can_frame& frame, uint16_t motor_id, float pos, float vel)
{
    //
}

void dm_speed_ctrl(can_frame& frame, uint16_t motor_id, float vel)
{
    //
}

void dm_save_pos_zero(can_frame& frame, uint16_t motor_id, uint16_t mode_id)
{
    //
}

void dm_clear_err(can_frame& frame, uint16_t motor_id, uint16_t mode_id)
{
    //
}

void dm_decode(const can_frame& frame, dm_motor_data_t& data)
{
    //
}
