#pragma once

#include "motor/motor_data.h"

#include <cstdint>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <sys/types.h>

enum class can_msg_type_lk : uint32_t
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

struct motor_state_lk
{
    uint8_t id {0};
    int8_t temperature {0};

    float   voltage {0};
    float   current {0};
    uint8_t motor_state {0};
    uint8_t error_state {0};

    float iq {0};
    float speed {0};
    float encoder {0};
    float encoder_raw {0};
    float encoder_offset {0};

    float ia {0};
    float ib {0};
    float ic {0};

    float position {0};
};

motor_state_lk lk_decode_frame(can_frame& frame);

void lk_get_motor_state_1(uint16_t id, can_frame& frame);
void lk_get_motor_state_2(uint16_t id, can_frame& frame);
void lk_get_motor_state_3(uint16_t id, can_frame& frame);
void lk_clear_motor_error(uint16_t id, can_frame& frame);

motor_state_lk lk_decode_motor_state_1(can_frame& frame);
motor_state_lk lk_decode_motor_state_2(can_frame& frame);
motor_state_lk lk_decode_motor_state_3(can_frame& frame);
motor_state_lk lk_decode_clear_motor_error(can_frame& frame);

void lk_disable_motor(uint16_t id, can_frame& frame);
void lk_enable_motor(uint16_t id, can_frame& frame);
void lk_stop_motor(uint16_t id, can_frame& frame);

motor_state_lk lk_decode_disable_motor(can_frame& frame);
motor_state_lk lk_decode_enable_motor(can_frame& frame);
motor_state_lk lk_decode_stop_motor(can_frame& frame);

void lk_torque_control(uint16_t id, can_frame& frame, float iq_control);
void lk_velocity_control_1(uint16_t id, can_frame& frame, float speed_control);
void lk_velocity_control_2(uint16_t id, can_frame& frame, float iq_control, float speed_control);
void lk_multi_pos_control_1(uint16_t id, can_frame& frame, float angle_control);
void lk_multi_pos_control_2(uint16_t id, can_frame& frame, float max_speed, float angle_control);
void lk_single_pos_control_1(uint16_t id, can_frame& frame, uint8_t spin_direction, float angle_control);
void lk_single_pos_control_2(uint16_t id, can_frame& frame, uint8_t direction, float max_speed, float angle_control);
// void lk_inc_pos_control_1(uint16_t id, can_frame& frame);
// void lk_inc_pos_control_2(uint16_t id, can_frame& frame);

motor_state_lk lk_decode_torque_control(can_frame& frame);
motor_state_lk lk_decode_velocity_control_1(can_frame& frame);
motor_state_lk lk_decode_velocity_control_2(can_frame& frame);
motor_state_lk lk_decode_multi_pos_control_1(can_frame& frame);
motor_state_lk lk_decode_multi_pos_control_2(can_frame& frame);
motor_state_lk lk_decode_single_pos_control_1(can_frame& frame);
motor_state_lk lk_decode_single_pos_control_2(can_frame& frame);
// motor_state_lk lk_decode_inc_pos_control_1(can_frame& frame);
// motor_state_lk lk_decode_inc_pos_control_2(can_frame& frame);

void lk_read_parameter(uint16_t id, can_frame& frame);
void lk_set_parameter(uint16_t id, can_frame& frame);
void lk_read_encoder(uint16_t id, can_frame& frame);

motor_state_lk lk_decode_read_parameter(can_frame& frame);
motor_state_lk lk_decode_set_parameter(can_frame& frame);
motor_state_lk lk_decode_read_encoder(can_frame& frame);

void lk_set_zero_encoder(uint16_t id, can_frame& frame, uint16_t encoder_offset);
void lk_set_zero_current(uint16_t id, can_frame& frame);

motor_state_lk lk_decode_set_zero_encoder(can_frame& frame);
motor_state_lk lk_decode_set_zero_current(can_frame& frame);

void lk_read_multi_pos(uint16_t id, can_frame& frame);
void lk_read_single_pos(uint16_t id, can_frame& frame);

motor_state_lk lk_decode_read_multi_pos(can_frame& frame);
motor_state_lk lk_decode_read_single_pos(can_frame& frame);
