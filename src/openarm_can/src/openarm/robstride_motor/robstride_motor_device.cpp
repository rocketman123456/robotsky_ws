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
#include <cstring>
#include <iostream>
#include <openarm/robstride_motor/robstride_motor_device.hpp>

namespace openarm::robstride_motor {

RSCANDevice::RSCANDevice(Motor& motor, bool use_fd)
    : canbus::CANDevice(receive_can_id(motor), receive_can_id(motor), receive_can_mask(), use_fd),
      motor_(motor),
      callback_mode_(CallbackMode::STATE),
      use_fd_(use_fd) {}

void RSCANDevice::callback(const can_frame& frame) {
    if (use_fd_) {
        std::cerr << "WARNING: Robstride classic CAN callback called while CAN-FD is enabled" << std::endl;
        return;
    }

    switch (callback_mode_) {
        case STATE: {
            StateResult result = CanPacketDecoder::parse_motor_state_frame(motor_, frame);
            if (result.valid) {
                motor_.update_state(result.position, result.velocity, result.torque, result.temperature, result.mode_status, result.fault_bits);
            }
            break;
        }
        case PARAM: {
            ParamResult result = CanPacketDecoder::parse_motor_param_frame(frame);
            if (result.valid) {
                motor_.set_temp_param(result.index, result.value);
            }
            break;
        }
        case IGNORE:
            return;
        default:
            break;
    }
}

void RSCANDevice::callback(const canfd_frame&) { std::cerr << "WARNING: Robstride private protocol uses classic CAN frames, not CAN-FD" << std::endl; }

can_frame RSCANDevice::create_can_frame(canid_t send_can_id, const std::vector<uint8_t>& data) {
    can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = send_can_id;
    frame.can_dlc = std::min(data.size(), sizeof(frame.data));
    std::copy(data.begin(), data.begin() + frame.can_dlc, frame.data);
    return frame;
}

canid_t RSCANDevice::receive_can_id(const Motor& motor) { return CAN_EFF_FLAG | (static_cast<canid_t>(motor.get_motor_id()) << 8) | motor.get_master_can_id(); }

canid_t RSCANDevice::receive_can_mask() { return CAN_EFF_FLAG | (static_cast<canid_t>(0xFF) << 8) | 0xFF; }

}  // namespace openarm::robstride_motor
