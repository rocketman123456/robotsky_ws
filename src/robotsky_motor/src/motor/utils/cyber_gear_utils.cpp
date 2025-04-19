#include "motor/utils/cyber_gear_utils.h"

#include <cstring>
#include <math.h>

static uint16_t s_masterid {0};

void cyber_ask_id(uint16_t id, can_frame& frame)
{
    uint16_t slaveid = s_masterid;

    uint16_t cyber_data[4] {0};
    cyber_data[0] = 0;
    cyber_data[1] = 0;
    cyber_data[2] = 0;
    cyber_data[3] = 0;
    cyber_encode(id, can_msg_type::GetId, frame, cyber_data, slaveid);
}

void cyber_set_can_id(uint16_t id, uint16_t target_id, can_frame& frame)
{
    uint16_t slaveid = s_masterid;
    slaveid |= ((uint32_t)target_id << 8);

    uint16_t cyber_data[4] {0};
    cyber_data[0] = 0;
    cyber_data[1] = 0;
    cyber_data[2] = 0;
    cyber_data[3] = 0;
    cyber_encode(id, can_msg_type::SetCANID, frame, cyber_data, slaveid);
}

void cyber_enable(uint16_t id, can_frame& frame)
{
    uint16_t slaveid = s_masterid;

    uint16_t cyber_data[4] {0};
    cyber_data[0] = 0;
    cyber_data[1] = 0;
    cyber_data[2] = 0;
    cyber_data[3] = 0;
    cyber_encode(id, can_msg_type::EnableMotor, frame, cyber_data, slaveid);
}

void cyber_disable(uint16_t id, can_frame& frame)
{
    uint16_t slaveid = s_masterid;

    uint16_t cyber_data[4] {0};
    cyber_data[0] = 0;
    cyber_data[1] = 0;
    cyber_data[2] = 0;
    cyber_data[3] = 0;
    cyber_encode(id, can_msg_type::DisableMotor, frame, cyber_data, slaveid);
}

void cyber_set_zero(uint16_t id, can_frame& frame)
{
    uint16_t slaveid = s_masterid;

    uint16_t cyber_data[4] {0};
    cyber_data[0] = 0x0100;
    cyber_data[1] = 0;
    cyber_data[2] = 0;
    cyber_data[3] = 0;
    cyber_encode(id, can_msg_type::SetZero, frame, cyber_data, slaveid);
}

void cyber_mixed_control(uint16_t id, float pos_des, float vel_des, float kp, float kd, float tau_des, can_frame& frame)
{
    uint16_t slaveid = (uint16_t)((tau_des + 12.0) / 24.0 * 65535.0);

    uint16_t cyber_data[4] {0};
    cyber_data[0] = (uint16_t)((pos_des + 4 * M_PI) / (8 * M_PI) * 65535.0);
    cyber_data[1] = (uint16_t)((vel_des + 30.0) / 60.0 * 65535.0);
    cyber_data[2] = (uint16_t)(kp / 500.0 * 65535.0);
    cyber_data[3] = (uint16_t)(kd / 5.0 * 65535.0);

    cyber_encode(id, can_msg_type::MixContorl, frame, cyber_data, slaveid);
}

void cyber_set_torque_limit(uint16_t id, float limit, can_frame& frame)
{
    uint32_t longdata = 0;
    longdata          = *(uint32_t*)&limit;
    uint8_t byte[4]   = {0};
    byte[0]           = (longdata & 0xFF000000) >> 24;
    byte[1]           = (longdata & 0x00FF0000) >> 16;
    byte[2]           = (longdata & 0x0000FF00) >> 8;
    byte[3]           = (longdata & 0x000000FF);

    uint16_t slaveid = s_masterid;

    uint16_t cyber_data[4] {0};
    cyber_data[0] = 0X0B70;
    cyber_data[1] = 0;
    cyber_data[2] = ((uint16_t)byte[3] << 8) | ((uint16_t)byte[2]);
    cyber_data[3] = ((uint16_t)byte[1] << 8) | ((uint16_t)byte[0]);

    cyber_encode(id, can_msg_type::SetZero, frame, cyber_data, slaveid);
}

void cyber_get_single_param(uint16_t id, uint16_t index, can_frame& frame)
{
    uint16_t slaveid = s_masterid;

    uint16_t cyber_data[4] {0};
    cyber_data[0] = index;
    cyber_data[1] = 0;
    cyber_data[2] = 0;
    cyber_data[3] = 0;

    cyber_encode(id, can_msg_type::ReadParam, frame, cyber_data, slaveid);
}

void cyber_encode(uint16_t id, can_msg_type type, can_frame& frame, uint16_t* cyber_data, uint16_t slave_id)
{
    // printf("can id : %x \n", id);
    frame.can_id = ((uint32_t)type) << 24;
    frame.can_id |= ((uint32_t)slave_id) << 8;
    frame.can_id |= ((uint32_t)id);
    frame.can_id |= CAN_EFF_FLAG;
    frame.can_dlc = 8;
    frame.data[0] = (uint8_t)(cyber_data[0] >> 8);
    frame.data[1] = (uint8_t)(cyber_data[0] & 0xff);
    frame.data[2] = (uint8_t)(cyber_data[1] >> 8);
    frame.data[3] = (uint8_t)(cyber_data[1] & 0xff);
    frame.data[4] = (uint8_t)(cyber_data[2] >> 8);
    frame.data[5] = (uint8_t)(cyber_data[2] & 0xff);
    frame.data[6] = (uint8_t)(cyber_data[3] >> 8);
    frame.data[7] = (uint8_t)(cyber_data[3] & 0xff);
}

bool cyber_get_feedback(const can_frame& frame, cyber_motor_data_t& data)
{
    uint32_t can_id = frame.can_id;
    if (can_id & CAN_EFF_FLAG)
    {
        can_id &= CAN_EFF_MASK;
    }
    uint16_t slaveid = (uint16_t)((can_id << 8) >> 16);
    uint16_t fbid    = (uint16_t)(can_id & 0xff);
    uint8_t  fbtype  = (uint8_t)((can_id << 3) >> 27);

    uint16_t cyber_data[4] {0};
    cyber_data[0] = ((uint16_t)frame.data[0] << 8) | (uint16_t)frame.data[1];
    cyber_data[1] = ((uint16_t)frame.data[2] << 8) | (uint16_t)frame.data[3];
    cyber_data[2] = ((uint16_t)frame.data[4] << 8) | (uint16_t)frame.data[5];
    cyber_data[3] = ((uint16_t)frame.data[6] << 8) | (uint16_t)frame.data[7];

    // cyber_decode(frame);
    if (fbtype == 2)
    {
        data.can_id = (slaveid & 0xff);
        data.pos    = ((double)cyber_data[0] - 32767.0) / 32768.0 * 4.0 * M_PI;
        data.vel    = ((double)cyber_data[1] - 32767.0) / 32768.0 * 30.0;
        data.tau    = ((double)cyber_data[2] - 32767.0) / 32768.0 * 12.0;
        data.tmp    = (double)cyber_data[3] / 10.0;
        return true;
    }
    else
    {
        return false;
    }
}

float cyber_decode_float_param(can_frame& frame)
{
    float data;

    uint8_t cyber_data[4] {0};
    cyber_data[3] = frame.data[7];
    cyber_data[2] = frame.data[6];
    cyber_data[1] = frame.data[5];
    cyber_data[0] = frame.data[4];
    memcpy(&data, cyber_data, sizeof(float));
    // printf("data: %f\n", data);

    return data;
}
