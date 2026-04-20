#include "can/old/can_driver.h"

#include <cassert>

void CanDriver::initialize(const std::vector<CanInitInfo>& infos)
{
    _interfaces.clear();
    _interfaces.reserve(infos.size());

    for (const auto& info : infos)
    {
        auto can_interface = std::make_shared<CANInterface>();
        can_interface->initialize(info);
        _interfaces.push_back(can_interface);
    }
}

void CanDriver::finalize()
{
    for (const auto& can_interface : _interfaces)
    {
        if (can_interface)
        {
            can_interface->finalize();
        }
    }
}

int CanDriver::getSocket(int index)
{
    assert(index >= 0 && index < static_cast<int>(_interfaces.size()) && "can index out of range");
    return _interfaces[index] ? _interfaces[index]->getSocketFd() : -1;
}

void CanDriver::send(int can_index, can_frame& frame)
{
    assert(can_index >= 0 && can_index < static_cast<int>(_interfaces.size()) && "can index out of range");
    _interfaces[can_index]->send(frame);
}

void CanDriver::receive(int can_index, can_frame& frame)
{
    assert(can_index >= 0 && can_index < static_cast<int>(_interfaces.size()) && "can index out of range");
    _interfaces[can_index]->receive(frame);
}
