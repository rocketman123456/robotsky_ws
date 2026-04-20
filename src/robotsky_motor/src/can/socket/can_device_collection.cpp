#include "can/socket/can_device_collection.h"

namespace robotsky_motor::canbus
{

CANDeviceCollection::CANDeviceCollection(CANSocket& can_socket)
    : can_socket_(can_socket)
{
}

void CANDeviceCollection::add_device(const std::shared_ptr<CANDevice>& device)
{
    if (!device)
    {
        return;
    }

    devices_[device->get_recv_can_id()] = device;
}

void CANDeviceCollection::remove_device(const std::shared_ptr<CANDevice>& device)
{
    if (!device)
    {
        return;
    }

    const auto it = devices_.find(device->get_recv_can_id());
    if (it != devices_.end())
    {
        devices_.erase(it);
    }
}

void CANDeviceCollection::dispatch_frame_callback(can_frame& frame)
{
    const auto it = devices_.find(frame.can_id);
    if (it != devices_.end())
    {
        it->second->callback(frame);
    }
}

void CANDeviceCollection::dispatch_frame_callback(canfd_frame& frame)
{
    const auto it = devices_.find(frame.can_id);
    if (it != devices_.end())
    {
        it->second->callback(frame);
    }
}

} // namespace robotsky_motor::canbus
