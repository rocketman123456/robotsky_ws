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

#include <cstdint>
#include <map>

#include "robstride_motor_constants.hpp"

namespace openarm::robstride_motor {

class Motor {
    friend class RSCANDevice;
    friend class RSDeviceCollection;

public:
    Motor(MotorType motor_type, uint8_t motor_id, uint8_t master_can_id = DEFAULT_MASTER_CAN_ID);

    double get_position() const { return state_q_; }
    double get_velocity() const { return state_dq_; }
    double get_torque() const { return state_tau_; }
    double get_temperature() const { return state_temperature_; }
    uint8_t get_mode_status() const { return state_mode_status_; }
    uint8_t get_fault_bits() const { return state_fault_bits_; }
    bool has_fault() const { return state_fault_bits_ != 0; }

    uint8_t get_motor_id() const { return motor_id_; }
    uint8_t get_master_can_id() const { return master_can_id_; }
    MotorType get_motor_type() const { return motor_type_; }
    bool is_enabled() const { return enabled_; }

    double get_param(uint16_t index) const;

    static LimitParam get_limit_param(MotorType motor_type);

protected:
    void update_state(double q, double dq, double tau, double temperature, uint8_t mode_status, uint8_t fault_bits);
    void set_enabled(bool enabled);
    void set_temp_param(uint16_t index, double value);

    MotorType motor_type_;
    uint8_t motor_id_;
    uint8_t master_can_id_;
    bool enabled_;

    double state_q_, state_dq_, state_tau_, state_temperature_;
    uint8_t state_mode_status_;
    uint8_t state_fault_bits_;

    std::map<uint16_t, double> temp_param_dict_;
};

}  // namespace openarm::robstride_motor
