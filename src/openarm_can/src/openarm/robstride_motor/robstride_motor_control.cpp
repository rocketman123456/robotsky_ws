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

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <openarm/robstride_motor/robstride_motor_control.hpp>

namespace openarm::robstride_motor {

CANPacket CanPacketEncoder::create_get_id_command(const Motor& motor) {
    return {build_extended_can_id(CommunicationType::GET_ID, motor.get_master_can_id(), motor.get_motor_id()), pack_empty_data()};
}

CANPacket CanPacketEncoder::create_enable_command(const Motor& motor) {
    return {build_extended_can_id(CommunicationType::MOTOR_ENABLE, motor.get_master_can_id(), motor.get_motor_id()), pack_empty_data()};
}

CANPacket CanPacketEncoder::create_disable_command(const Motor& motor, bool clear_error) {
    auto data = pack_empty_data();
    data[0] = clear_error ? 0x01 : 0x00;
    return {build_extended_can_id(CommunicationType::MOTOR_STOP, motor.get_master_can_id(), motor.get_motor_id()), data};
}

CANPacket CanPacketEncoder::create_clear_error_command(const Motor& motor) { return create_disable_command(motor, true); }

CANPacket CanPacketEncoder::create_set_zero_command(const Motor& motor) {
    auto data = pack_empty_data();
    data[0] = 0x01;
    return {build_extended_can_id(CommunicationType::SET_MECHANICAL_ZERO, motor.get_master_can_id(), motor.get_motor_id()), data};
}

CANPacket CanPacketEncoder::create_change_can_id_command(const Motor& motor, uint8_t new_motor_id) {
    return {build_extended_can_id(CommunicationType::SET_CAN_ID, static_cast<uint16_t>((new_motor_id << 8) | motor.get_master_can_id()), motor.get_motor_id()),
            pack_empty_data()};
}

CANPacket CanPacketEncoder::create_save_parameters_command(const Motor& motor) {
    return {build_extended_can_id(CommunicationType::SAVE_PARAMETERS, motor.get_master_can_id(), motor.get_motor_id()),
            {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}};
}

CANPacket CanPacketEncoder::create_mit_control_command(const Motor& motor, const MITParam& mit_param) {
    const LimitParam limits = Motor::get_limit_param(motor.get_motor_type());
    const uint16_t torque_uint = double_to_uint(mit_param.tau, limits.tMin, limits.tMax, 16);

    return {build_extended_can_id(CommunicationType::MOTION_CONTROL, torque_uint, motor.get_motor_id()),
            pack_mit_control_data(motor.get_motor_type(), mit_param)};
}

CANPacket CanPacketEncoder::create_set_control_mode_command(const Motor& motor, ControlMode mode) {
    return create_set_parameter_uint8_command(motor, static_cast<uint16_t>(ParameterIndex::RUN_MODE), static_cast<uint8_t>(mode));
}

CANPacket CanPacketEncoder::create_query_param_command(const Motor& motor, uint16_t index) {
    return {build_extended_can_id(CommunicationType::GET_SINGLE_PARAMETER, motor.get_master_can_id(), motor.get_motor_id()), pack_index_data(index)};
}

CANPacket CanPacketEncoder::create_set_parameter_float_command(const Motor& motor, uint16_t index, float value) {
    return {build_extended_can_id(CommunicationType::SET_SINGLE_PARAMETER, motor.get_master_can_id(), motor.get_motor_id()),
            pack_float_parameter_data(index, value)};
}

CANPacket CanPacketEncoder::create_set_parameter_uint8_command(const Motor& motor, uint16_t index, uint8_t value) {
    return {build_extended_can_id(CommunicationType::SET_SINGLE_PARAMETER, motor.get_master_can_id(), motor.get_motor_id()),
            pack_uint8_parameter_data(index, value)};
}

canid_t CanPacketEncoder::build_extended_can_id(CommunicationType communication_type, uint16_t data_field, uint8_t target_id) {
    return CAN_EFF_FLAG | (static_cast<canid_t>(communication_type) << 24) | (static_cast<canid_t>(data_field) << 8) | target_id;
}

std::vector<uint8_t> CanPacketEncoder::pack_empty_data() { return {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; }

std::vector<uint8_t> CanPacketEncoder::pack_index_data(uint16_t index) {
    return {static_cast<uint8_t>(index & 0xFF), static_cast<uint8_t>((index >> 8) & 0xFF), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
}

std::vector<uint8_t> CanPacketEncoder::pack_float_parameter_data(uint16_t index, float value) {
    std::array<uint8_t, 4> value_bytes{};
    std::memcpy(value_bytes.data(), &value, sizeof(float));

    return {static_cast<uint8_t>(index & 0xFF),
            static_cast<uint8_t>((index >> 8) & 0xFF),
            0x00,
            0x00,
            value_bytes[0],
            value_bytes[1],
            value_bytes[2],
            value_bytes[3]};
}

std::vector<uint8_t> CanPacketEncoder::pack_uint8_parameter_data(uint16_t index, uint8_t value) {
    return {static_cast<uint8_t>(index & 0xFF), static_cast<uint8_t>((index >> 8) & 0xFF), 0x00, 0x00, value, 0x00, 0x00, 0x00};
}

std::vector<uint8_t> CanPacketEncoder::pack_mit_control_data(MotorType motor_type, const MITParam& mit_param) {
    const LimitParam limits = Motor::get_limit_param(motor_type);

    const uint16_t q_uint = double_to_uint(mit_param.q, limits.pMin, limits.pMax, 16);
    const uint16_t dq_uint = double_to_uint(mit_param.dq, limits.vMin, limits.vMax, 16);
    const uint16_t kp_uint = double_to_uint(mit_param.kp, limits.kpMin, limits.kpMax, 16);
    const uint16_t kd_uint = double_to_uint(mit_param.kd, limits.kdMin, limits.kdMax, 16);

    return {static_cast<uint8_t>((q_uint >> 8) & 0xFF),  static_cast<uint8_t>(q_uint & 0xFF),         static_cast<uint8_t>((dq_uint >> 8) & 0xFF),
            static_cast<uint8_t>(dq_uint & 0xFF),        static_cast<uint8_t>((kp_uint >> 8) & 0xFF), static_cast<uint8_t>(kp_uint & 0xFF),
            static_cast<uint8_t>((kd_uint >> 8) & 0xFF), static_cast<uint8_t>(kd_uint & 0xFF)};
}

double CanPacketEncoder::limit_min_max(double x, double min, double max) { return std::max(min, std::min(x, max)); }

uint16_t CanPacketEncoder::double_to_uint(double x, double x_min, double x_max, int bits) {
    x = limit_min_max(x, x_min, x_max);
    double span = x_max - x_min;
    double data_norm = (x - x_min) / span;
    return static_cast<uint16_t>(data_norm * ((1 << bits) - 1));
}

CommunicationType CanPacketDecoder::get_communication_type(canid_t can_id) { return static_cast<CommunicationType>((can_id & CAN_EFF_MASK) >> 24); }

uint8_t CanPacketDecoder::get_target_id(canid_t can_id) { return static_cast<uint8_t>(can_id & 0xFF); }

StateResult CanPacketDecoder::parse_motor_state_frame(const Motor& motor, const can_frame& frame) {
    if (frame.can_dlc < 8) {
        std::cerr << "Warning: Skipping Robstride state data less than 8 bytes" << std::endl;
        return {0, 0, 0, 0, 0, 0, 0, false};
    }

    const auto communication_type = get_communication_type(frame.can_id);
    if (communication_type != CommunicationType::MOTOR_FEEDBACK && communication_type != CommunicationType::ACTIVE_REPORT) {
        return {0, 0, 0, 0, 0, 0, 0, false};
    }

    const uint8_t target_id = get_target_id(frame.can_id);
    if (target_id != motor.get_master_can_id()) {
        return {0, 0, 0, 0, 0, 0, 0, false};
    }

    const uint16_t data_field = static_cast<uint16_t>((frame.can_id & 0x00FFFF00) >> 8);
    const uint8_t motor_id = static_cast<uint8_t>(data_field & 0xFF);
    if (motor_id != motor.get_motor_id()) {
        return {0, 0, 0, 0, 0, 0, 0, false};
    }

    const uint8_t mode_status = static_cast<uint8_t>((data_field >> 14) & 0x03);
    const uint8_t fault_bits = static_cast<uint8_t>((data_field >> 8) & 0x3F);
    const uint16_t q_uint = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
    const uint16_t dq_uint = (static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3];
    const uint16_t tau_uint = (static_cast<uint16_t>(frame.data[4]) << 8) | frame.data[5];
    const uint16_t temperature_uint = (static_cast<uint16_t>(frame.data[6]) << 8) | frame.data[7];

    const LimitParam limits = Motor::get_limit_param(motor.get_motor_type());
    const double recv_q = uint_to_double(q_uint, limits.pMin, limits.pMax, 16);
    const double recv_dq = uint_to_double(dq_uint, limits.vMin, limits.vMax, 16);
    const double recv_tau = uint_to_double(tau_uint, limits.tMin, limits.tMax, 16);
    const double recv_temperature = static_cast<double>(temperature_uint) * 0.1;

    return {motor_id, recv_q, recv_dq, recv_tau, recv_temperature, mode_status, fault_bits, true};
}

ParamResult CanPacketDecoder::parse_motor_param_frame(const can_frame& frame) {
    if (frame.can_dlc < 8) {
        return {0, NAN, false};
    }

    if (get_communication_type(frame.can_id) != CommunicationType::GET_SINGLE_PARAMETER) {
        return {0, NAN, false};
    }

    const uint16_t status = static_cast<uint16_t>((frame.can_id & 0x00FF0000) >> 16);
    if (status != 0x00) {
        return {0, NAN, false};
    }

    const uint16_t index = data_index(frame);
    if (parameter_is_uint8(index)) {
        return {index, static_cast<double>(frame.data[4]), true};
    }
    if (parameter_is_int16(index)) {
        int16_t value = 0;
        std::memcpy(&value, &frame.data[4], sizeof(value));
        return {index, static_cast<double>(value), true};
    }

    std::array<uint8_t, 4> float_bytes = {frame.data[4], frame.data[5], frame.data[6], frame.data[7]};
    return {index, static_cast<double>(uint8s_to_float(float_bytes)), true};
}

double CanPacketDecoder::uint_to_double(uint16_t x, double min, double max, int bits) {
    double span = max - min;
    double data_norm = static_cast<double>(x) / ((1 << bits) - 1);
    return data_norm * span + min;
}

float CanPacketDecoder::uint8s_to_float(const std::array<uint8_t, 4>& bytes) {
    float value;
    std::memcpy(&value, bytes.data(), sizeof(float));
    return value;
}

uint16_t CanPacketDecoder::data_index(const can_frame& frame) { return static_cast<uint16_t>(frame.data[0]) | (static_cast<uint16_t>(frame.data[1]) << 8); }

bool CanPacketDecoder::parameter_is_uint8(uint16_t index) { return index == static_cast<uint16_t>(ParameterIndex::RUN_MODE); }

bool CanPacketDecoder::parameter_is_int16(uint16_t index) { return index == static_cast<uint16_t>(ParameterIndex::ROTATION); }

}  // namespace openarm::robstride_motor
