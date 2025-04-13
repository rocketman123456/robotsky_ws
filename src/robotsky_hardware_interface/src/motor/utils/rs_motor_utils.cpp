#include "motor/utils/rs_motor_utils.h"

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
    frame.can_id  = RS_MotorEnable << 24 | RS_MASTER_ID << 8 | motor_id;
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
    frame.can_id  = RS_MotorStop << 24 | RS_MASTER_ID << 8 | motor_id;
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
