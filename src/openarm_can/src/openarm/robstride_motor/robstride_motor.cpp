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

#include <cmath>
#include <openarm/robstride_motor/robstride_motor.hpp>
#include <stdexcept>
#include <string>

namespace openarm::robstride_motor {

Motor::Motor(MotorType motor_type, uint8_t motor_id, uint8_t master_can_id)
    : motor_type_(motor_type),
      motor_id_(motor_id),
      master_can_id_(master_can_id),
      enabled_(false),
      state_q_(0.0),
      state_dq_(0.0),
      state_tau_(0.0),
      state_temperature_(0.0),
      state_mode_status_(0),
      state_fault_bits_(0) {}

double Motor::get_param(uint16_t index) const {
    auto it = temp_param_dict_.find(index);
    if (it == temp_param_dict_.end()) {
        return NAN;
    }
    return it->second;
}

LimitParam Motor::get_limit_param(MotorType motor_type) {
    size_t index = static_cast<size_t>(motor_type);
    if (index >= MOTOR_LIMIT_PARAMS.size()) {
        throw std::invalid_argument("Invalid Robstride motor type: " + std::to_string(static_cast<int>(motor_type)));
    }
    return MOTOR_LIMIT_PARAMS[index];
}

void Motor::update_state(double q, double dq, double tau, double temperature, uint8_t mode_status, uint8_t fault_bits) {
    state_q_ = q;
    state_dq_ = dq;
    state_tau_ = tau;
    state_temperature_ = temperature;
    state_mode_status_ = mode_status;
    state_fault_bits_ = fault_bits;
}

void Motor::set_enabled(bool enabled) { enabled_ = enabled; }

void Motor::set_temp_param(uint16_t index, double value) { temp_param_dict_[index] = value; }

}  // namespace openarm::robstride_motor
