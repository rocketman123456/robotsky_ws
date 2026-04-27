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

#include <deque>
#include <memory>
#include <vector>

#include "../canbus/can_device_collection.hpp"
#include "robstride_motor_control.hpp"
#include "robstride_motor_device.hpp"

namespace openarm::robstride_motor {

class RSDeviceCollection {
public:
    explicit RSDeviceCollection(canbus::CANSocket& can_socket);
    virtual ~RSDeviceCollection() = default;

    void add_motor(MotorType motor_type, uint8_t motor_id, uint8_t master_can_id = DEFAULT_MASTER_CAN_ID,
                   ControlMode control_mode = ControlMode::MIT);

    void enable_all();
    void disable_all();
    void clear_error_all();
    void set_zero(int i);
    void set_zero_all();
    void save_parameters_all();
    void set_callback_mode_all(CallbackMode callback_mode);

    void query_param_one(int i, uint16_t index);
    void query_param_all(uint16_t index);
    void set_parameter_float_one(int i, uint16_t index, float value);
    void set_parameter_uint8_one(int i, uint16_t index, uint8_t value);

    void set_control_mode_one(int i, ControlMode mode);
    void set_control_mode_all(ControlMode mode);

    void mit_control_one(int i, const MITParam& mit_param);
    void mit_control_all(const std::vector<MITParam>& mit_params);

    std::vector<Motor> get_motors() const;
    Motor get_motor(int i) const;
    canbus::CANDeviceCollection& get_device_collection() { return *device_collection_; }

protected:
    void send_command_to_device(std::shared_ptr<RSCANDevice> rs_device, const CANPacket& packet);
    std::vector<std::shared_ptr<RSCANDevice>> get_rs_devices() const;

    canbus::CANSocket& can_socket_;
    std::unique_ptr<canbus::CANDeviceCollection> device_collection_;
    std::deque<Motor> motors_;
    std::vector<std::shared_ptr<RSCANDevice>> rs_devices_;
};

}  // namespace openarm::robstride_motor
