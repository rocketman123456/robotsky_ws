// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <linux/can.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

#include "robstride_motor.hpp"
#include "robstride_motor_constants.hpp"

namespace openarm::robstride_motor {

struct ParamResult {
    uint16_t index;
    double value;
    bool valid;
};

struct StateResult {
    uint8_t motor_id;
    double position;
    double velocity;
    double torque;
    double temperature;
    uint8_t mode_status;
    uint8_t fault_bits;
    bool valid;
};

struct CANPacket {
    canid_t send_can_id;
    std::vector<uint8_t> data;
};

struct MITParam {
    double kp;
    double kd;
    double q;
    double dq;
    double tau;
};

class CanPacketEncoder {
public:
    static CANPacket create_get_id_command(const Motor& motor);
    static CANPacket create_enable_command(const Motor& motor);
    static CANPacket create_disable_command(const Motor& motor, bool clear_error = false);
    static CANPacket create_clear_error_command(const Motor& motor);
    static CANPacket create_set_zero_command(const Motor& motor);
    static CANPacket create_change_can_id_command(const Motor& motor, uint8_t new_motor_id);
    static CANPacket create_save_parameters_command(const Motor& motor);
    static CANPacket create_mit_control_command(const Motor& motor, const MITParam& mit_param);
    static CANPacket create_set_control_mode_command(const Motor& motor, ControlMode mode);
    static CANPacket create_query_param_command(const Motor& motor, uint16_t index);
    static CANPacket create_set_parameter_float_command(const Motor& motor, uint16_t index, float value);
    static CANPacket create_set_parameter_uint8_command(const Motor& motor, uint16_t index, uint8_t value);

    static canid_t build_extended_can_id(CommunicationType communication_type, uint16_t data_field, uint8_t target_id);

private:
    static std::vector<uint8_t> pack_empty_data();
    static std::vector<uint8_t> pack_index_data(uint16_t index);
    static std::vector<uint8_t> pack_float_parameter_data(uint16_t index, float value);
    static std::vector<uint8_t> pack_uint8_parameter_data(uint16_t index, uint8_t value);
    static std::vector<uint8_t> pack_mit_control_data(MotorType motor_type, const MITParam& mit_param);

    static double limit_min_max(double x, double min, double max);
    static uint16_t double_to_uint(double x, double x_min, double x_max, int bits);
};

class CanPacketDecoder {
public:
    static CommunicationType get_communication_type(canid_t can_id);
    static uint8_t get_target_id(canid_t can_id);
    static StateResult parse_motor_state_frame(const Motor& motor, const can_frame& frame);
    static ParamResult parse_motor_param_frame(const can_frame& frame);

private:
    static double uint_to_double(uint16_t x, double min, double max, int bits);
    static float uint8s_to_float(const std::array<uint8_t, 4>& bytes);
    static uint16_t data_index(const can_frame& frame);
    static bool parameter_is_uint8(uint16_t index);
    static bool parameter_is_int16(uint16_t index);
};

}  // namespace openarm::robstride_motor
