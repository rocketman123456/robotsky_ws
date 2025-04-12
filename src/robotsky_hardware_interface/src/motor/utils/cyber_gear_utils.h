#pragma once

#include "motor/motor_data.h"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>

// to avoid bug, please don't use in multi-thread

enum class can_msg_type : uint32_t
{
    GetId         = 0x00,
    MixContorl    = 0x01,
    MotorFeedback = 0x02,
    EnableMotor   = 0x03,
    DisableMotor  = 0x04,
    SetZero       = 0x06,
    SetCANID      = 0x07,
    ReadParam     = 0x11,
    WriteParam    = 0x12,
    ErrorFeedback = 0x15,
};

void cyber_ask_id(uint16_t id, can_frame& frame);
void cyber_set_can_id(uint16_t id, uint16_t target_id, can_frame& frame);
void cyber_set_single_param(uint16_t id, uint16_t index, can_frame& frame);
void cyber_enable(uint16_t id, can_frame& frame);
void cyber_disable(uint16_t id, can_frame& frame);
void cyber_set_zero(uint16_t id, can_frame& frame);
void cyber_mixed_control(uint16_t id, motor_data_t& data, can_frame& frame);
void cyber_set_torque_limit(uint16_t id, float limit, can_frame& frame);

void cyber_get_single_param(uint16_t id, uint16_t index, can_frame& frame);
bool cyber_get_feedback(const can_frame& frame, motor_data_t& data);

float cyber_decode_float_param(can_frame& frame);
void  cyber_encode(uint16_t id, can_msg_type type, can_frame& frame);
void  cyber_decode(const can_frame& frame);