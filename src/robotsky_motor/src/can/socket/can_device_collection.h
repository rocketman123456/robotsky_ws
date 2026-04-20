#pragma once

#include "can/socket/can_device.h"
#include "can/socket/can_socket.h"

#include <map>
#include <memory>

namespace robotsky_motor::canbus
{

class CANDeviceCollection
{
public:
    explicit CANDeviceCollection(CANSocket& can_socket);
    ~CANDeviceCollection() = default;

    void add_device(const std::shared_ptr<CANDevice>& device);
    void remove_device(const std::shared_ptr<CANDevice>& device);
    void dispatch_frame_callback(can_frame& frame);
    void dispatch_frame_callback(canfd_frame& frame);

    const std::map<canid_t, std::shared_ptr<CANDevice>>& get_devices() const { return devices_; }
    CANSocket&                                           get_can_socket() const { return can_socket_; }
    int                                                  get_socket_fd() const { return can_socket_.get_socket_fd(); }

private:
    CANSocket&                                 can_socket_;
    std::map<canid_t, std::shared_ptr<CANDevice>> devices_;
};

} // namespace robotsky_motor::canbus
