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

#include <array>
#include <cstddef>
#include <vector>

#include "../../canbus/can_socket.hpp"
#include "../../damiao_motor/dm_motor.hpp"
#include "../../damiao_motor/dm_motor_device_collection.hpp"

namespace openarm::can::socket {

class LegComponent : public damiao_motor::DMDeviceCollection {
public:
    static constexpr std::size_t kJointCountPerLeg = 4;

    enum JointIndex : std::size_t {
        ROLL = 0,
        HIP = 1,
        KNEE = 2,
        WHEEL = 3,
    };

    LegComponent(canbus::CANSocket& can_socket);
    ~LegComponent() = default;

    void init_motor_devices(const std::vector<damiao_motor::MotorType>& motor_types, const std::vector<uint32_t>& send_can_ids,
                            const std::vector<uint32_t>& recv_can_ids, bool use_fd,
                            const std::vector<damiao_motor::ControlMode>& control_modes = {});

    std::size_t leg_count() const { return motors_.size() / kJointCountPerLeg; }

    void mit_control_leg(std::size_t leg_index, const std::array<damiao_motor::MITParam, kJointCountPerLeg>& params);
    void set_leg_pose(std::size_t leg_index, double roll, double hip, double knee, double wheel, double kp = 20.0, double kd = 1.0);
    void set_wheel_velocity(std::size_t leg_index, double velocity_rad_s);
    void set_wheel_velocities(const std::vector<double>& velocities_rad_s);

private:
    std::vector<damiao_motor::Motor> motors_;

    std::size_t motor_index(std::size_t leg_index, JointIndex joint_index) const;
};

}  // namespace openarm::can::socket
