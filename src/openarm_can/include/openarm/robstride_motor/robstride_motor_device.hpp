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

#include "../canbus/can_device.hpp"
#include "robstride_motor.hpp"
#include "robstride_motor_control.hpp"

namespace openarm::robstride_motor {

enum CallbackMode {
    STATE,
    PARAM,
    IGNORE,
};

class RSCANDevice : public canbus::CANDevice {
public:
    explicit RSCANDevice(Motor& motor, bool use_fd = false);

    void callback(const can_frame& frame) override;
    void callback(const canfd_frame& frame) override;

    can_frame create_can_frame(canid_t send_can_id, const std::vector<uint8_t>& data);

    Motor& get_motor() { return motor_; }
    void set_callback_mode(CallbackMode callback_mode) { callback_mode_ = callback_mode; }
    ControlMode get_control_mode() const { return control_mode_; }
    void set_control_mode(ControlMode control_mode) { control_mode_ = control_mode; }

    static canid_t receive_can_id(const Motor& motor);
    static canid_t receive_can_mask();

private:
    Motor& motor_;
    CallbackMode callback_mode_;
    bool use_fd_;
    ControlMode control_mode_ = ControlMode::MIT;
};

}  // namespace openarm::robstride_motor
