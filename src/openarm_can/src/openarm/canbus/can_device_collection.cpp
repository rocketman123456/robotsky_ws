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
#include <openarm/canbus/can_device_collection.hpp>
#include <openarm/canbus/can_socket.hpp>

namespace openarm::canbus {
namespace {

bool frame_matches_device(canid_t frame_id, const std::shared_ptr<CANDevice>& device) {
    const canid_t device_id = device->get_recv_can_id();
    const canid_t device_mask = device->get_recv_can_mask();

    if ((frame_id & CAN_EFF_FLAG) != (device_id & CAN_EFF_FLAG)) {
        return false;
    }

    return (frame_id & device_mask) == (device_id & device_mask);
}

}  // namespace

CANDeviceCollection::CANDeviceCollection(CANSocket& can_socket) : can_socket_(can_socket) {}

CANDeviceCollection::~CANDeviceCollection() {}

void CANDeviceCollection::add_device(const std::shared_ptr<CANDevice>& device) {
    if (!device) return;

    // Add device to our collection
    canid_t device_id = device->get_recv_can_id();
    devices_[device_id] = device;
}

void CANDeviceCollection::remove_device(const std::shared_ptr<CANDevice>& device) {
    if (!device) return;

    canid_t device_id = device->get_recv_can_id();
    auto it = devices_.find(device_id);
    if (it != devices_.end()) {
        // Remove from our collection
        devices_.erase(it);
    }
}

void CANDeviceCollection::dispatch_frame_callback(can_frame& frame) {
    for (const auto& [id, device] : devices_) {
        (void)id;
        if (frame_matches_device(frame.can_id, device)) {
            device->callback(frame);
            return;
        }
    }
    // Note: Silently ignore frames for unknown devices (this is normal in CAN
    // networks)
}

void CANDeviceCollection::dispatch_frame_callback(canfd_frame& frame) {
    for (const auto& [id, device] : devices_) {
        (void)id;
        if (frame_matches_device(frame.can_id, device)) {
            device->callback(frame);
            return;
        }
    }
    // Note: Silently ignore frames for unknown devices (this is normal in CAN
    // networks)
}

}  // namespace openarm::canbus
