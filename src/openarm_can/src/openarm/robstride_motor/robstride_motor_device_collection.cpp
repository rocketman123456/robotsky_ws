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

#include <iostream>
#include <openarm/robstride_motor/robstride_motor_device_collection.hpp>
#include <stdexcept>

namespace openarm::robstride_motor {

RSDeviceCollection::RSDeviceCollection(canbus::CANSocket& can_socket)
    : can_socket_(can_socket), device_collection_(std::make_unique<canbus::CANDeviceCollection>(can_socket_)) {}

void RSDeviceCollection::add_motor(MotorType motor_type, uint8_t motor_id, uint8_t master_can_id, ControlMode control_mode) {
    motors_.emplace_back(motor_type, motor_id, master_can_id);
    auto motor_device = std::make_shared<RSCANDevice>(motors_.back(), false);
    motor_device->set_control_mode(control_mode);
    get_device_collection().add_device(motor_device);
    rs_devices_.push_back(motor_device);
}

void RSDeviceCollection::enable_all() {
    for (auto rs_device : get_rs_devices()) {
        CANPacket enable_packet = CanPacketEncoder::create_enable_command(rs_device->get_motor());
        send_command_to_device(rs_device, enable_packet);
        rs_device->get_motor().set_enabled(true);
    }
}

void RSDeviceCollection::disable_all() {
    for (auto rs_device : get_rs_devices()) {
        CANPacket disable_packet = CanPacketEncoder::create_disable_command(rs_device->get_motor(), false);
        send_command_to_device(rs_device, disable_packet);
        rs_device->get_motor().set_enabled(false);
    }
}

void RSDeviceCollection::clear_error_all() {
    for (auto rs_device : get_rs_devices()) {
        CANPacket clear_error_packet = CanPacketEncoder::create_clear_error_command(rs_device->get_motor());
        send_command_to_device(rs_device, clear_error_packet);
    }
}

void RSDeviceCollection::set_zero(int i) {
    auto rs_device = get_rs_devices().at(i);
    CANPacket zero_packet = CanPacketEncoder::create_set_zero_command(rs_device->get_motor());
    send_command_to_device(rs_device, zero_packet);
}

void RSDeviceCollection::set_zero_all() {
    for (auto rs_device : get_rs_devices()) {
        CANPacket zero_packet = CanPacketEncoder::create_set_zero_command(rs_device->get_motor());
        send_command_to_device(rs_device, zero_packet);
    }
}

void RSDeviceCollection::save_parameters_all() {
    for (auto rs_device : get_rs_devices()) {
        CANPacket save_packet = CanPacketEncoder::create_save_parameters_command(rs_device->get_motor());
        send_command_to_device(rs_device, save_packet);
    }
}

void RSDeviceCollection::set_callback_mode_all(CallbackMode callback_mode) {
    for (auto rs_device : get_rs_devices()) {
        rs_device->set_callback_mode(callback_mode);
    }
}

void RSDeviceCollection::query_param_one(int i, uint16_t index) {
    auto rs_device = get_rs_devices().at(i);
    CANPacket query_packet = CanPacketEncoder::create_query_param_command(rs_device->get_motor(), index);
    send_command_to_device(rs_device, query_packet);
}

void RSDeviceCollection::query_param_all(uint16_t index) {
    for (auto rs_device : get_rs_devices()) {
        CANPacket query_packet = CanPacketEncoder::create_query_param_command(rs_device->get_motor(), index);
        send_command_to_device(rs_device, query_packet);
    }
}

void RSDeviceCollection::set_parameter_float_one(int i, uint16_t index, float value) {
    auto rs_device = get_rs_devices().at(i);
    CANPacket set_packet = CanPacketEncoder::create_set_parameter_float_command(rs_device->get_motor(), index, value);
    send_command_to_device(rs_device, set_packet);
}

void RSDeviceCollection::set_parameter_uint8_one(int i, uint16_t index, uint8_t value) {
    auto rs_device = get_rs_devices().at(i);
    CANPacket set_packet = CanPacketEncoder::create_set_parameter_uint8_command(rs_device->get_motor(), index, value);
    send_command_to_device(rs_device, set_packet);
}

void RSDeviceCollection::set_control_mode_one(int i, ControlMode mode) {
    auto rs_device = get_rs_devices().at(i);
    rs_device->set_control_mode(mode);
    CANPacket mode_packet = CanPacketEncoder::create_set_control_mode_command(rs_device->get_motor(), mode);
    send_command_to_device(rs_device, mode_packet);
}

void RSDeviceCollection::set_control_mode_all(ControlMode mode) {
    auto rs_devices = get_rs_devices();
    for (size_t i = 0; i < rs_devices.size(); i++) {
        set_control_mode_one(static_cast<int>(i), mode);
    }
}

void RSDeviceCollection::mit_control_one(int i, const MITParam& mit_param) {
    auto rs_device = get_rs_devices().at(i);
    if (rs_device->get_control_mode() != ControlMode::MIT) {
        std::cerr << "WARNING: Robstride MIT control rejected; motor not in MIT mode." << std::endl;
        return;
    }

    CANPacket mit_packet = CanPacketEncoder::create_mit_control_command(rs_device->get_motor(), mit_param);
    send_command_to_device(rs_device, mit_packet);
}

void RSDeviceCollection::mit_control_all(const std::vector<MITParam>& mit_params) {
    for (size_t i = 0; i < mit_params.size(); i++) {
        mit_control_one(static_cast<int>(i), mit_params[i]);
    }
}

std::vector<Motor> RSDeviceCollection::get_motors() const {
    std::vector<Motor> motors;
    for (auto rs_device : get_rs_devices()) {
        motors.push_back(rs_device->get_motor());
    }
    return motors;
}

Motor RSDeviceCollection::get_motor(int i) const { return get_rs_devices().at(i)->get_motor(); }

void RSDeviceCollection::send_command_to_device(std::shared_ptr<RSCANDevice> rs_device, const CANPacket& packet) {
    if (can_socket_.is_canfd_enabled()) {
        std::cerr << "WARNING: Robstride private protocol does not support CAN-FD; sending classic "
                     "CAN frame"
                  << std::endl;
    }

    can_frame frame = rs_device->create_can_frame(packet.send_can_id, packet.data);
    can_socket_.write_can_frame(frame);
}

std::vector<std::shared_ptr<RSCANDevice>> RSDeviceCollection::get_rs_devices() const { return rs_devices_; }

}  // namespace openarm::robstride_motor
