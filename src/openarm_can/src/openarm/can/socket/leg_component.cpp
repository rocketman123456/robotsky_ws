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

#include <linux/can.h>
#include <linux/can/raw.h>

#include <openarm/can/socket/leg_component.hpp>
#include <stdexcept>

namespace openarm::can::socket {

LegComponent::LegComponent(canbus::CANSocket& can_socket) : damiao_motor::DMDeviceCollection(can_socket) {}

void LegComponent::init_motor_devices(const std::vector<damiao_motor::MotorType>& motor_types, const std::vector<canid_t>& send_can_ids,
                                      const std::vector<canid_t>& recv_can_ids, bool use_fd,
                                      const std::vector<damiao_motor::ControlMode>& control_modes) {
    if (motor_types.size() % kJointCountPerLeg != 0) {
        throw std::invalid_argument("LegComponent motor count must be a multiple of 4.");
    }

    motors_.reserve(motor_types.size());

    for (std::size_t i = 0; i < motor_types.size(); i++) {
        motors_.emplace_back(motor_types[i], send_can_ids[i], recv_can_ids[i]);
        auto motor_device = std::make_shared<damiao_motor::DMCANDevice>(motors_.back(), CAN_SFF_MASK, use_fd);
        get_device_collection().add_device(motor_device);
    }

    if (!control_modes.empty()) {
        if (control_modes.size() == 1) {
            set_control_mode_all(control_modes[0]);
        } else if (control_modes.size() == motor_types.size()) {
            for (std::size_t i = 0; i < motor_types.size(); i++) {
                set_control_mode_one(static_cast<int>(i), control_modes[i]);
            }
        } else {
            throw std::invalid_argument("Control modes vector must have a single element or match the motor count.");
        }
    }
}

void LegComponent::mit_control_leg(std::size_t leg_index, const std::array<damiao_motor::MITParam, kJointCountPerLeg>& params) {
    for (std::size_t joint = 0; joint < kJointCountPerLeg; joint++) {
        mit_control_one(static_cast<int>(motor_index(leg_index, static_cast<JointIndex>(joint))), params[joint]);
    }
}

void LegComponent::set_leg_pose(std::size_t leg_index, double roll, double hip, double knee, double wheel, double kp, double kd) {
    mit_control_leg(leg_index, {damiao_motor::MITParam{kp, kd, roll, 0.0, 0.0}, damiao_motor::MITParam{kp, kd, hip, 0.0, 0.0},
                                damiao_motor::MITParam{kp, kd, knee, 0.0, 0.0}, damiao_motor::MITParam{0.0, kd, wheel, 0.0, 0.0}});
}

void LegComponent::set_wheel_velocity(std::size_t leg_index, double velocity_rad_s) {
    mit_control_one(static_cast<int>(motor_index(leg_index, WHEEL)), damiao_motor::MITParam{0.0, 0.0, 0.0, velocity_rad_s, 0.0});
}

void LegComponent::set_wheel_velocities(const std::vector<double>& velocities_rad_s) {
    if (velocities_rad_s.size() != leg_count()) {
        throw std::invalid_argument("Wheel velocity count must match leg count.");
    }

    for (std::size_t leg_index = 0; leg_index < velocities_rad_s.size(); leg_index++) {
        set_wheel_velocity(leg_index, velocities_rad_s[leg_index]);
    }
}

std::size_t LegComponent::motor_index(std::size_t leg_index, JointIndex joint_index) const {
    if (leg_index >= leg_count()) {
        throw std::out_of_range("Leg index out of range.");
    }

    return leg_index * kJointCountPerLeg + static_cast<std::size_t>(joint_index);
}

}  // namespace openarm::can::socket
