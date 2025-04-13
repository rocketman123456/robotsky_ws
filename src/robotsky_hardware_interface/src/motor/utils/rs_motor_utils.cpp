#include "motor/utils/rs_motor_utils.h"

#include <cstring>

float rs_uint16_to_float(uint16_t x, float x_min, float x_max, int bits)
{
    uint32_t span   = (1 << bits) - 1;
    float    offset = x_max - x_min;
    return offset * x / span + x_min;
}

int rs_float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span   = x_max - x_min;
    float offset = x_min;
    if (x > x_max)
        x = x_max;
    else if (x < x_min)
        x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float rs_byte_to_float(uint8_t* bytedata)
{
    uint32_t data       = bytedata[7] << 24 | bytedata[6] << 16 | bytedata[5] << 8 | bytedata[4];
    float    data_float = *(float*)(&data);
    return data_float;
}

void rs_enable_motor_mode(can_frame& frame, uint16_t motor_id)
{
    frame.can_id = RS_MotorEnable << 24 | RS_MASTER_ID << 8 | motor_id;
    frame.can_id |= CAN_EFF_FLAG;
    frame.can_dlc = 8;

    frame.data[0] = 0x00;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
}

void rs_disable_motor_mode(can_frame& frame, uint16_t motor_id, uint16_t clear_error)
{
    frame.can_id = RS_MotorStop << 24 | RS_MASTER_ID << 8 | motor_id;
    frame.can_id |= CAN_EFF_FLAG;
    frame.can_dlc = 8;

    frame.data[0] = 0x00;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
}

void rs_set_motor_parameter(can_frame& frame, uint16_t motor_id, uint16_t index, float value, uint8_t value_mode)
{
    frame.can_id = RS_SetSingleParameter << 24 | RS_MASTER_ID << 8 | motor_id;
    frame.can_id |= CAN_EFF_FLAG;
    frame.can_dlc = 8;

    frame.data[0] = index;
    frame.data[1] = index >> 8;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;

    if (value_mode == 'p')
    {
        memcpy(&frame.data[4], &value, 4);
    }
    else if (value_mode == 'j')
    {
        frame.data[4] = (uint8_t)value;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
    }
}

void rs_get_motor_parameter(can_frame& frame, uint16_t motor_id, uint16_t index)
{
    frame.can_id = RS_GetSingleParameter << 24 | RS_MASTER_ID << 8 | motor_id;
    frame.can_id |= CAN_EFF_FLAG;
    frame.can_dlc = 8;

    frame.data[0] = index;
    frame.data[1] = index >> 8;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
}

void rs_mit_ctrl(can_frame& frame, uint16_t motor_id, float pos, float vel, float kp, float kd, float tau)
{
    frame.can_id = RS_MotionControl << 24 | rs_float_to_uint(tau, RS_T_MIN, RS_T_MAX, 16) << 8 | motor_id;
    frame.can_id |= CAN_EFF_FLAG;
    frame.can_dlc = 8;

    frame.data[0] = rs_float_to_uint(pos, RS_P_MIN, RS_P_MAX, 16) >> 8;
    frame.data[1] = rs_float_to_uint(pos, RS_P_MIN, RS_P_MAX, 16);
    frame.data[2] = rs_float_to_uint(vel, RS_V_MIN, RS_V_MAX, 16) >> 8;
    frame.data[3] = rs_float_to_uint(vel, RS_V_MIN, RS_V_MAX, 16);
    frame.data[4] = rs_float_to_uint(kp, RS_KP_MIN, RS_KP_MAX, 16) >> 8;
    frame.data[5] = rs_float_to_uint(kp, RS_KP_MIN, RS_KP_MAX, 16);
    frame.data[6] = rs_float_to_uint(kd, RS_KD_MIN, RS_KD_MAX, 16) >> 8;
    frame.data[7] = rs_float_to_uint(kd, RS_KD_MIN, RS_KD_MAX, 16);
}

void rs_pos_speed_ctrl(can_frame& frame, uint16_t motor_id, float pos, float vel)
{
    //
}

void rs_speed_ctrl(can_frame& frame, uint16_t motor_id, float vel)
{
    //
}

void rs_save_pos_zero(can_frame& frame, uint16_t motor_id)
{
    frame.can_id = RS_SetPosZero << 24 | RS_MASTER_ID << 8 | motor_id;
    frame.can_id |= CAN_EFF_FLAG;
    frame.can_dlc = 8;

    frame.data[0] = 0x01;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
}
