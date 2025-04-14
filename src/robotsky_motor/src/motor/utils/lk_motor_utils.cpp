#include "motor/utils/lk_motor_utils.h"

#include <spdlog/spdlog.h>

#define _USE_MATH_DEFINES
#include <math.h>

constexpr uint32_t DEVICE_STD_ID = 0x140;
constexpr uint32_t DEVICE_RTN_ID = 320; // 0x180;

motor_state_lk lk_decode_frame(can_frame& frame)
{
    // for (int i = 0; i < 8; ++i)
    // {
    //     spdlog::info("{} : {}", i, frame.data[i]);
    // }

    switch (frame.data[0])
    {
        case 0x9A:
            return lk_decode_motor_state_1(frame);
        case 0x9B:
            return lk_decode_clear_motor_error(frame);
        case 0x9C:
            return lk_decode_motor_state_2(frame);
        case 0x9D:
            return lk_decode_motor_state_3(frame);
        case 0x80:
            return lk_decode_disable_motor(frame);
        case 0x88:
            return lk_decode_enable_motor(frame);
        case 0x81:
            return lk_decode_stop_motor(frame);
        case 0xA1:
            return lk_decode_torque_control(frame);
        case 0xA2:
            return lk_decode_velocity_control_1(frame);
        case 0xAD:
            return lk_decode_velocity_control_2(frame);
        case 0xA3:
            return lk_decode_multi_pos_control_1(frame);
        case 0xA4:
            return lk_decode_multi_pos_control_2(frame);
        case 0xA5:
            return lk_decode_single_pos_control_1(frame);
        case 0xA6:
            return lk_decode_single_pos_control_2(frame);
        case 0x90:
            return lk_decode_read_encoder(frame);
        case 0x91:
            return lk_decode_set_zero_encoder(frame);
        case 0x19:
            return lk_decode_set_zero_current(frame);
        case 0x92:
            return lk_decode_read_multi_pos(frame);
        case 0x94:
            return lk_decode_read_single_pos(frame);
        default:
            spdlog::warn("error return code {}", frame.data[0]);
            return {};
    }
}

motor_state_lk lk_decode_type_2(uint8_t type, can_frame& frame)
{
    if (frame.data[0] != type)
    {
        spdlog::warn("error return code {}, get {}", type, frame.data[0]);
        return {};
    }
    motor_state_lk state;

    state.id          = frame.can_id - DEVICE_RTN_ID;
    state.temperature = frame.data[1];

    // spdlog::warn("get motor id {}, frame id {}", state.id, frame.can_id);

    int16_t  iq      = (static_cast<int16_t>(frame.data[3]) << 8) | static_cast<int16_t>(frame.data[2]);
    int16_t  speed   = (static_cast<int16_t>(frame.data[5]) << 8) | static_cast<int16_t>(frame.data[4]);
    uint16_t encoder = (static_cast<uint16_t>(frame.data[7]) << 8) | static_cast<uint16_t>(frame.data[6]);

    state.iq      = iq * 33.0 / 4096.0;
    state.speed   = speed * M_PI / 180.0;
    state.encoder = encoder;

    return state;
}

void lk_get_motor_state_1(uint16_t id, can_frame& frame)
{
    frame.can_id  = DEVICE_STD_ID + id;
    frame.can_dlc = 8;
    frame.data[0] = static_cast<uint8_t>(0x9A);
    frame.data[1] = static_cast<uint8_t>(0);
    frame.data[2] = static_cast<uint8_t>(0);
    frame.data[3] = static_cast<uint8_t>(0);
    frame.data[4] = static_cast<uint8_t>(0);
    frame.data[5] = static_cast<uint8_t>(0);
    frame.data[6] = static_cast<uint8_t>(0);
    frame.data[7] = static_cast<uint8_t>(0);
}

motor_state_lk lk_decode_motor_state_1(can_frame& frame)
{
    if (frame.data[0] != 0x9A)
    {
        spdlog::warn("error return code 0x9A, get {}", frame.data[0]);
        return {};
    }
    motor_state_lk state;

    state.id          = frame.can_id - DEVICE_RTN_ID;
    state.temperature = frame.data[1];

    uint16_t volt = (static_cast<uint16_t>(frame.data[3]) << 8) | static_cast<uint16_t>(frame.data[2]);
    uint16_t curr = (static_cast<uint16_t>(frame.data[5]) << 8) | static_cast<uint16_t>(frame.data[4]);

    state.voltage     = volt * 0.01;
    state.current     = curr * 0.01;
    state.motor_state = frame.data[6];
    state.error_state = frame.data[7];

    return state;
}

void lk_get_motor_state_2(uint16_t id, can_frame& frame)
{
    frame.can_id  = DEVICE_STD_ID + id;
    frame.can_dlc = 8;
    frame.data[0] = static_cast<uint8_t>(0x9C);
    frame.data[1] = static_cast<uint8_t>(0);
    frame.data[2] = static_cast<uint8_t>(0);
    frame.data[3] = static_cast<uint8_t>(0);
    frame.data[4] = static_cast<uint8_t>(0);
    frame.data[5] = static_cast<uint8_t>(0);
    frame.data[6] = static_cast<uint8_t>(0);
    frame.data[7] = static_cast<uint8_t>(0);
}

motor_state_lk lk_decode_motor_state_2(can_frame& frame) { return lk_decode_type_2(0x9C, frame); }

void lk_get_motor_state_3(uint16_t id, can_frame& frame)
{
    frame.can_id  = DEVICE_STD_ID + id;
    frame.can_dlc = 8;
    frame.data[0] = static_cast<uint8_t>(0x9D);
    frame.data[1] = static_cast<uint8_t>(0);
    frame.data[2] = static_cast<uint8_t>(0);
    frame.data[3] = static_cast<uint8_t>(0);
    frame.data[4] = static_cast<uint8_t>(0);
    frame.data[5] = static_cast<uint8_t>(0);
    frame.data[6] = static_cast<uint8_t>(0);
    frame.data[7] = static_cast<uint8_t>(0);
}

motor_state_lk lk_decode_motor_state_3(can_frame& frame)
{
    if (frame.data[0] != 0x9D)
    {
        spdlog::warn("error return code 0x9D, get {}", frame.data[0]);
        return {};
    }
    motor_state_lk state;

    state.id          = frame.can_id - DEVICE_RTN_ID;
    state.temperature = frame.data[1];

    uint16_t ia = (static_cast<uint16_t>(frame.data[3]) << 8) | static_cast<uint16_t>(frame.data[2]);
    uint16_t ib = (static_cast<uint16_t>(frame.data[5]) << 8) | static_cast<uint16_t>(frame.data[4]);
    uint16_t ic = (static_cast<uint16_t>(frame.data[7]) << 8) | static_cast<uint16_t>(frame.data[6]);

    state.ia = ia * 33.0 / 4096.0;
    state.ib = ib * 33.0 / 4096.0;
    state.ic = ic * 33.0 / 4096.0;

    return state;
}

void lk_clear_motor_error(uint16_t id, can_frame& frame)
{
    frame.can_id  = DEVICE_STD_ID + id;
    frame.can_dlc = 8;
    frame.data[0] = static_cast<uint8_t>(0x9B);
    frame.data[1] = static_cast<uint8_t>(0);
    frame.data[2] = static_cast<uint8_t>(0);
    frame.data[3] = static_cast<uint8_t>(0);
    frame.data[4] = static_cast<uint8_t>(0);
    frame.data[5] = static_cast<uint8_t>(0);
    frame.data[6] = static_cast<uint8_t>(0);
    frame.data[7] = static_cast<uint8_t>(0);
}

motor_state_lk lk_decode_clear_motor_error(can_frame& frame)
{
    if (frame.data[0] != 0x9B)
    {
        spdlog::warn("error return code 0x9B, get {}", frame.data[0]);
        return {};
    }
    motor_state_lk state;

    state.id          = frame.can_id - DEVICE_RTN_ID;
    state.temperature = frame.data[1];

    uint16_t volt = (static_cast<uint16_t>(frame.data[3]) << 8) | static_cast<uint16_t>(frame.data[2]);
    uint16_t curr = (static_cast<uint16_t>(frame.data[5]) << 8) | static_cast<uint16_t>(frame.data[4]);

    state.voltage     = volt * 0.01;
    state.current     = curr * 0.01;
    state.motor_state = frame.data[6];
    state.error_state = frame.data[7];

    return state;
}

//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------

void lk_disable_motor(uint16_t id, can_frame& frame)
{
    frame.can_id  = DEVICE_STD_ID + id;
    frame.can_dlc = 8;
    frame.data[0] = static_cast<uint8_t>(0x80);
    frame.data[1] = static_cast<uint8_t>(0);
    frame.data[2] = static_cast<uint8_t>(0);
    frame.data[3] = static_cast<uint8_t>(0);
    frame.data[4] = static_cast<uint8_t>(0);
    frame.data[5] = static_cast<uint8_t>(0);
    frame.data[6] = static_cast<uint8_t>(0);
    frame.data[7] = static_cast<uint8_t>(0);
}

motor_state_lk lk_decode_disable_motor(can_frame& frame)
{
    if (frame.data[0] != 0x80)
    {
        spdlog::warn("error return code 0x80, get {}", frame.data[0]);
        return {};
    }

    motor_state_lk state;

    state.id = frame.can_id - DEVICE_RTN_ID;

    return state;
}

void lk_enable_motor(uint16_t id, can_frame& frame)
{
    frame.can_id  = DEVICE_STD_ID + id;
    frame.can_dlc = 8;
    frame.data[0] = static_cast<uint8_t>(0x88);
    frame.data[1] = static_cast<uint8_t>(0);
    frame.data[2] = static_cast<uint8_t>(0);
    frame.data[3] = static_cast<uint8_t>(0);
    frame.data[4] = static_cast<uint8_t>(0);
    frame.data[5] = static_cast<uint8_t>(0);
    frame.data[6] = static_cast<uint8_t>(0);
    frame.data[7] = static_cast<uint8_t>(0);
}

motor_state_lk lk_decode_enable_motor(can_frame& frame)
{
    if (frame.data[0] != 0x88)
    {
        spdlog::warn("error return code 0x88, get {}", frame.data[0]);
        return {};
    }

    motor_state_lk state;

    state.id = frame.can_id - DEVICE_RTN_ID;

    return state;
}

void lk_stop_motor(uint16_t id, can_frame& frame)
{
    frame.can_id  = DEVICE_STD_ID + id;
    frame.can_dlc = 8;
    frame.data[0] = static_cast<uint8_t>(0x81);
    frame.data[1] = static_cast<uint8_t>(0);
    frame.data[2] = static_cast<uint8_t>(0);
    frame.data[3] = static_cast<uint8_t>(0);
    frame.data[4] = static_cast<uint8_t>(0);
    frame.data[5] = static_cast<uint8_t>(0);
    frame.data[6] = static_cast<uint8_t>(0);
    frame.data[7] = static_cast<uint8_t>(0);
}

motor_state_lk lk_decode_stop_motor(can_frame& frame)
{
    if (frame.data[0] != 0x81)
    {
        spdlog::warn("error return code 0x81, get {}", frame.data[0]);
        return {};
    }

    motor_state_lk state;

    state.id = frame.can_id - DEVICE_RTN_ID;

    return state;
}

//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------

void lk_torque_control(uint16_t id, can_frame& frame, float iq_control)
{
    int16_t iq = static_cast<int16_t>(iq_control * 4096.0 / 33.0);

    frame.can_id  = DEVICE_STD_ID + id;
    frame.can_dlc = 8;
    frame.data[0] = static_cast<uint8_t>(0xA1);
    frame.data[1] = static_cast<uint8_t>(0);
    frame.data[2] = static_cast<uint8_t>(0);
    frame.data[3] = static_cast<uint8_t>(0);
    frame.data[4] = static_cast<uint8_t>(iq & 0x00ff);
    frame.data[5] = static_cast<uint8_t>((iq & 0xff00) >> 8);
    frame.data[6] = static_cast<uint8_t>(0);
    frame.data[7] = static_cast<uint8_t>(0);
}

motor_state_lk lk_decode_torque_control(can_frame& frame) { return lk_decode_type_2(0xA1, frame); }

void lk_velocity_control_1(uint16_t id, can_frame& frame, float speed_control)
{
    int32_t speed = static_cast<int32_t>(speed_control * 100.0 * 180.0 / M_PI);

    frame.can_id  = DEVICE_STD_ID + id;
    frame.can_dlc = 8;
    frame.data[0] = static_cast<uint8_t>(0xA2);
    frame.data[1] = static_cast<uint8_t>(0);
    frame.data[2] = static_cast<uint8_t>(2048 & 0x00ff);
    frame.data[3] = static_cast<uint8_t>((2048 & 0xff00) >> 8);
    frame.data[4] = static_cast<uint8_t>(speed & 0x000000ff);
    frame.data[5] = static_cast<uint8_t>((speed & 0x0000ff00) >> 8);
    frame.data[6] = static_cast<uint8_t>((speed & 0x00ff0000) >> 16);
    frame.data[7] = static_cast<uint8_t>((speed & 0xff000000) >> 24);
}

motor_state_lk lk_decode_velocity_control_1(can_frame& frame) { return lk_decode_type_2(0xA2, frame); }

void lk_velocity_control_2(uint16_t id, can_frame& frame, float iq_control, float speed_control)
{
    int16_t iq    = static_cast<int16_t>(iq_control * 4096.0 / 33.0);
    int32_t speed = static_cast<int32_t>(100.0 * speed_control * 180.0 / M_PI);

    frame.can_id  = DEVICE_STD_ID + id;
    frame.can_dlc = 8;
    frame.data[0] = static_cast<uint8_t>(0xA2);
    frame.data[1] = static_cast<uint8_t>(0);
    frame.data[2] = static_cast<uint8_t>(iq & 0x00ff);
    frame.data[3] = static_cast<uint8_t>((iq & 0xff00) >> 8);
    frame.data[4] = static_cast<uint8_t>(speed & 0x000000ff);
    frame.data[5] = static_cast<uint8_t>((speed & 0x0000ff00) >> 8);
    frame.data[6] = static_cast<uint8_t>((speed & 0x00ff0000) >> 16);
    frame.data[7] = static_cast<uint8_t>((speed & 0xff000000) >> 24);
}

motor_state_lk lk_decode_velocity_control_2(can_frame& frame) { return lk_decode_type_2(0xAD, frame); }

void lk_multi_pos_control_1(uint16_t id, can_frame& frame, float angle_control)
{
    int32_t angle = static_cast<int32_t>(100.0 * angle_control * 180.0 / M_PI);

    frame.can_id  = DEVICE_STD_ID + id;
    frame.can_dlc = 8;
    frame.data[0] = static_cast<uint8_t>(0xA3);
    frame.data[1] = static_cast<uint8_t>(0);
    frame.data[2] = static_cast<uint8_t>(0);
    frame.data[3] = static_cast<uint8_t>(0);
    frame.data[4] = static_cast<uint8_t>(angle & 0x000000ff);
    frame.data[5] = static_cast<uint8_t>((angle & 0x0000ff00) >> 8);
    frame.data[6] = static_cast<uint8_t>((angle & 0x00ff0000) >> 16);
    frame.data[7] = static_cast<uint8_t>((angle & 0xff000000) >> 24);
}

motor_state_lk lk_decode_multi_pos_control_1(can_frame& frame) { return lk_decode_type_2(0xA3, frame); }

void lk_multi_pos_control_2(uint16_t id, can_frame& frame, float max_speed, float angle_control)
{
    uint16_t speed = static_cast<uint16_t>(max_speed * 180.0 / M_PI);
    int32_t  angle = static_cast<int32_t>(100.0 * angle_control * 180.0 / M_PI);

    frame.can_id  = DEVICE_STD_ID + id;
    frame.can_dlc = 8;
    frame.data[0] = static_cast<uint8_t>(0xA4);
    frame.data[1] = static_cast<uint8_t>(0);
    frame.data[2] = static_cast<uint8_t>(speed & 0x00ff);
    frame.data[3] = static_cast<uint8_t>((speed & 0xff00) >> 8);
    frame.data[4] = static_cast<uint8_t>(angle & 0x000000ff);
    frame.data[5] = static_cast<uint8_t>((angle & 0x0000ff00) >> 8);
    frame.data[6] = static_cast<uint8_t>((angle & 0x00ff0000) >> 16);
    frame.data[7] = static_cast<uint8_t>((angle & 0xff000000) >> 24);
}

motor_state_lk lk_decode_multi_pos_control_2(can_frame& frame) { return lk_decode_type_2(0xA4, frame); }

void lk_single_pos_control_1(uint16_t id, can_frame& frame, uint8_t spin_direction, float angle_control)
{
    uint32_t angle = static_cast<uint32_t>(100.0 * angle_control * 180.0 / M_PI);

    frame.can_id  = DEVICE_STD_ID + id;
    frame.can_dlc = 8;
    frame.data[0] = static_cast<uint8_t>(0xA5);
    frame.data[1] = static_cast<uint8_t>(spin_direction);
    frame.data[2] = static_cast<uint8_t>(0);
    frame.data[3] = static_cast<uint8_t>(0);
    frame.data[4] = static_cast<uint8_t>(angle & 0x000000ff);
    frame.data[5] = static_cast<uint8_t>((angle & 0x0000ff00) >> 8);
    frame.data[6] = static_cast<uint8_t>((angle & 0x00ff0000) >> 16);
    frame.data[7] = static_cast<uint8_t>((angle & 0xff000000) >> 24);
}

motor_state_lk lk_decode_single_pos_control_1(can_frame& frame) { return lk_decode_type_2(0xA5, frame); }

void lk_single_pos_control_2(uint16_t id, can_frame& frame, uint8_t direction, float max_speed, float angle_control)
{
    uint16_t speed = static_cast<uint16_t>(max_speed * 180.0 / M_PI);
    uint32_t angle = static_cast<uint32_t>(100.0 * angle_control * 180.0 / M_PI);

    frame.can_id  = DEVICE_STD_ID + id;
    frame.can_dlc = 8;
    frame.data[0] = static_cast<uint8_t>(0xA6);
    frame.data[1] = static_cast<uint8_t>(direction);
    frame.data[2] = static_cast<uint8_t>(speed & 0x00ff);
    frame.data[3] = static_cast<uint8_t>((speed & 0xff00) >> 8);
    frame.data[4] = static_cast<uint8_t>(angle & 0x000000ff);
    frame.data[5] = static_cast<uint8_t>((angle & 0x0000ff00) >> 8);
    frame.data[6] = static_cast<uint8_t>((angle & 0x00ff0000) >> 16);
    frame.data[7] = static_cast<uint8_t>((angle & 0xff000000) >> 24);
}

motor_state_lk lk_decode_single_pos_control_2(can_frame& frame) { return lk_decode_type_2(0xA6, frame); }

// void           lk_inc_pos_control_1(uint16_t id, can_frame& frame);
// motor_state_lk lk_decode_inc_pos_control_1(uint16_t id, can_frame& frame);

// void           lk_inc_pos_control_2(uint16_t id, can_frame& frame);
// motor_state_lk lk_decode_inc_pos_control_2(uint16_t id, can_frame& frame);

//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------

void lk_read_parameter(uint16_t id, can_frame& frame, uint8_t index)
{
    frame.can_id  = DEVICE_STD_ID + id;
    frame.can_dlc = 8;
    frame.data[0] = static_cast<uint8_t>(0xB0);
    frame.data[1] = static_cast<uint8_t>(index);
    frame.data[2] = static_cast<uint8_t>(0);
    frame.data[3] = static_cast<uint8_t>(0);
    frame.data[4] = static_cast<uint8_t>(0);
    frame.data[5] = static_cast<uint8_t>(0);
    frame.data[6] = static_cast<uint8_t>(0);
    frame.data[7] = static_cast<uint8_t>(0);
}

motor_state_lk lk_decode_read_parameter(can_frame& frame)
{
    // TODO
    return {};
}

void lk_set_parameter(uint16_t id, can_frame& frame, uint8_t index)
{
    // TODO
    frame.can_id  = DEVICE_STD_ID + id;
    frame.can_dlc = 8;
    frame.data[0] = static_cast<uint8_t>(0xB1);
    frame.data[1] = static_cast<uint8_t>(index);
    frame.data[2] = static_cast<uint8_t>(0);
    frame.data[3] = static_cast<uint8_t>(0);
    frame.data[4] = static_cast<uint8_t>(0);
    frame.data[5] = static_cast<uint8_t>(0);
    frame.data[6] = static_cast<uint8_t>(0);
    frame.data[7] = static_cast<uint8_t>(0);
}

motor_state_lk lk_decode_set_parameter(can_frame& frame)
{
    // TODO
    return {};
}

//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------

void lk_read_encoder(uint16_t id, can_frame& frame)
{
    frame.can_id  = DEVICE_STD_ID + id;
    frame.can_dlc = 8;
    frame.data[0] = static_cast<uint8_t>(0x90);
    frame.data[1] = static_cast<uint8_t>(0);
    frame.data[2] = static_cast<uint8_t>(0);
    frame.data[3] = static_cast<uint8_t>(0);
    frame.data[4] = static_cast<uint8_t>(0);
    frame.data[5] = static_cast<uint8_t>(0);
    frame.data[6] = static_cast<uint8_t>(0);
    frame.data[7] = static_cast<uint8_t>(0);
}

motor_state_lk lk_decode_read_encoder(can_frame& frame)
{
    if (frame.data[0] != 0x90)
    {
        spdlog::warn("error return code 0x90, get {}", frame.data[0]);
        return {};
    }
    motor_state_lk state;

    state.id = frame.can_id - DEVICE_RTN_ID;

    uint16_t encoder        = (static_cast<uint16_t>(frame.data[3]) << 8) | static_cast<uint16_t>(frame.data[2]);
    uint16_t encoder_raw    = (static_cast<uint16_t>(frame.data[5]) << 8) | static_cast<uint16_t>(frame.data[4]);
    uint16_t encoder_offset = (static_cast<uint16_t>(frame.data[7]) << 8) | static_cast<uint16_t>(frame.data[6]);

    state.encoder        = encoder;
    state.encoder_raw    = encoder_raw;
    state.encoder_offset = encoder_offset;

    return state;
}

void lk_set_zero_encoder(uint16_t id, can_frame& frame, uint16_t encoder_offset)
{
    frame.can_id  = DEVICE_STD_ID + id;
    frame.can_dlc = 8;
    frame.data[0] = static_cast<uint8_t>(0x91);
    frame.data[1] = static_cast<uint8_t>(0);
    frame.data[2] = static_cast<uint8_t>(0);
    frame.data[3] = static_cast<uint8_t>(0);
    frame.data[4] = static_cast<uint8_t>(0);
    frame.data[5] = static_cast<uint8_t>(0);
    frame.data[6] = static_cast<uint8_t>(encoder_offset & 0x00ff);
    frame.data[7] = static_cast<uint8_t>((encoder_offset & 0xff00) >> 8);
}

motor_state_lk lk_decode_set_zero_encoder(can_frame& frame)
{
    if (frame.data[0] != 0x91)
    {
        spdlog::warn("error return code 0x91, get {}", frame.data[0]);
        return {};
    }
    motor_state_lk state;

    state.id = frame.can_id - DEVICE_RTN_ID;

    uint16_t encoder_offset = (static_cast<uint16_t>(frame.data[7]) << 8) | static_cast<uint16_t>(frame.data[6]);

    state.encoder_offset = encoder_offset;

    return state;
}

void lk_set_zero_current(uint16_t id, can_frame& frame)
{
    frame.can_id  = DEVICE_STD_ID + id;
    frame.can_dlc = 8;
    frame.data[0] = static_cast<uint8_t>(0x19);
    frame.data[1] = static_cast<uint8_t>(0);
    frame.data[2] = static_cast<uint8_t>(0);
    frame.data[3] = static_cast<uint8_t>(0);
    frame.data[4] = static_cast<uint8_t>(0);
    frame.data[5] = static_cast<uint8_t>(0);
    frame.data[6] = static_cast<uint8_t>(0);
    frame.data[7] = static_cast<uint8_t>(0);
}

motor_state_lk lk_decode_set_zero_current(can_frame& frame)
{
    if (frame.data[0] != 0x19)
    {
        spdlog::warn("error return code 0x19, get {}", frame.data[0]);
        return {};
    }
    motor_state_lk state;

    state.id = frame.can_id - DEVICE_RTN_ID;

    uint16_t encoder_offset = (static_cast<uint16_t>(frame.data[7]) << 8) | static_cast<uint16_t>(frame.data[6]);

    state.encoder_offset = encoder_offset;

    return state;
}

//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------

void lk_read_multi_pos(uint16_t id, can_frame& frame)
{
    frame.can_id  = DEVICE_STD_ID + id;
    frame.can_dlc = 8;
    frame.data[0] = static_cast<uint8_t>(0x92);
    frame.data[1] = static_cast<uint8_t>(0);
    frame.data[2] = static_cast<uint8_t>(0);
    frame.data[3] = static_cast<uint8_t>(0);
    frame.data[4] = static_cast<uint8_t>(0);
    frame.data[5] = static_cast<uint8_t>(0);
    frame.data[6] = static_cast<uint8_t>(0);
    frame.data[7] = static_cast<uint8_t>(0);
}

motor_state_lk lk_decode_read_multi_pos(can_frame& frame)
{
    if (frame.data[0] != 0x92)
    {
        spdlog::warn("error return code 0x92, get {}", frame.data[0]);
        return {};
    }
    motor_state_lk state;

    state.id = frame.can_id - DEVICE_RTN_ID;

    int64_t angle = (static_cast<int64_t>(frame.data[7]) << 48) | (static_cast<int64_t>(frame.data[6]) << 40) | (static_cast<int64_t>(frame.data[5]) << 32) |
                    (static_cast<int64_t>(frame.data[4]) << 24) | (static_cast<int64_t>(frame.data[3]) << 16) | (static_cast<int64_t>(frame.data[2]) << 8) |
                    (static_cast<int64_t>(frame.data[1]));

    state.position = angle * 0.01 * M_PI / 180.0;

    return state;
}

void lk_read_single_pos(uint16_t id, can_frame& frame)
{
    frame.can_id  = DEVICE_STD_ID + id;
    frame.can_dlc = 8;
    frame.data[0] = static_cast<uint8_t>(0x94);
    frame.data[1] = static_cast<uint8_t>(0);
    frame.data[2] = static_cast<uint8_t>(0);
    frame.data[3] = static_cast<uint8_t>(0);
    frame.data[4] = static_cast<uint8_t>(0);
    frame.data[5] = static_cast<uint8_t>(0);
    frame.data[6] = static_cast<uint8_t>(0);
    frame.data[7] = static_cast<uint8_t>(0);
}

motor_state_lk lk_decode_read_single_pos(can_frame& frame)
{
    if (frame.data[0] != 0x94)
    {
        spdlog::warn("error return code 0x94, get {}", frame.data[0]);
        return {};
    }
    motor_state_lk state;

    state.id = frame.can_id - DEVICE_RTN_ID;

    // clang-format off
    uint32_t angle = (static_cast<uint32_t>(frame.data[7]) << 24) | 
                     (static_cast<uint32_t>(frame.data[6]) << 16) | 
                     (static_cast<uint32_t>(frame.data[5]) << 8) |
                     (static_cast<uint32_t>(frame.data[4]));
    // clang-format on

    state.position = angle * 0.01 * M_PI / 180.0;

    return state;
}
