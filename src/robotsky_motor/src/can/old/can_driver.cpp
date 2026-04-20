#include "can/old/can_driver.h"

#include <assert.h>

void CanDriver::initialize(const std::vector<CanInitInfo>& infos)
{
    finalize();

    for (const auto& info : infos)
    {
        auto can_interface = std::make_unique<CANInterface>();
        can_interface->initialize(info);
        _interfaces.push_back(std::move(can_interface));
    }
}

void CanDriver::finalize()
{
    for (auto& can_interface : _interfaces)
    {
        if (can_interface)
        {
            can_interface->finalize();
        }
    }

    _interfaces.clear();
}

int CanDriver::getSocket(int index)
{
    assert(index >= 0 && static_cast<size_t>(index) < _interfaces.size());
    return _interfaces[index] ? _interfaces[index]->socketFD() : -1;
}

bool CanDriver::send(int can_index, const can_frame& frame)
{
    assert(can_index >= 0 && static_cast<size_t>(can_index) < _interfaces.size() && "can index out of range");
    return _interfaces[can_index]->send(frame);
}

bool CanDriver::receive(int can_index, can_frame& frame, int timeout_us)
{
    assert(can_index >= 0 && static_cast<size_t>(can_index) < _interfaces.size() && "can index out of range");
    return _interfaces[can_index]->receive(frame, timeout_us);
}

bool CanDriver::receiveMatching(int can_index, can_frame& frame, const FrameMatcher& matcher, int timeout_us)
{
    assert(can_index >= 0 && static_cast<size_t>(can_index) < _interfaces.size() && "can index out of range");
    return _interfaces[can_index]->receiveMatching(frame, matcher, timeout_us);
}

bool CanDriver::requestResponse(int can_index, const can_frame& tx, can_frame& rx, const FrameMatcher& matcher, int timeout_us)
{
    assert(can_index >= 0 && static_cast<size_t>(can_index) < _interfaces.size() && "can index out of range");
    return _interfaces[can_index]->requestResponse(tx, rx, matcher, timeout_us);
}
