#include "can/can_interface.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>

void CANInterface::initialize(const CanInitInfo& info)
{
    _can_name  = info.can_port;
    _enable_fd = info.enable_fd;
    _socket    = std::make_unique<robotsky_motor::canbus::CANSocket>(info.can_port, info.enable_fd);
    spdlog::info("open can: {}{}", _can_name, _enable_fd ? " (fd)" : "");
}

void CANInterface::finalize()
{
    if (_socket)
    {
        spdlog::info("close can: {}", _can_name);
        _socket.reset();
    }
}

bool CANInterface::send(const can_frame& frame)
{
    if (!_socket || !_socket->is_initialized())
    {
        spdlog::error("CAN interface {} is not initialized", _can_name);
        return false;
    }

    if (!_socket->write_can_frame(frame))
    {
        spdlog::warn("Failed to write CAN frame on {}", _can_name);
        return false;
    }

    return true;
}

bool CANInterface::receive(can_frame& frame, int timeout_us)
{
    if (!_socket || !_socket->is_initialized())
    {
        return false;
    }

    if (!_socket->is_data_available(timeout_us))
    {
        return false;
    }

    return _socket->read_can_frame(frame);
}

bool CANInterface::receiveMatching(can_frame& frame, const FrameMatcher& matcher, int timeout_us)
{
    using Clock = std::chrono::steady_clock;

    if (!_socket || !_socket->is_initialized())
    {
        return false;
    }

    const auto deadline = Clock::now() + std::chrono::microseconds(std::max(timeout_us, 1));

    while (Clock::now() < deadline)
    {
        const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(deadline - Clock::now()).count();
        const int  slice_us  = static_cast<int>(std::max<long long>(1, std::min<long long>(remaining, 500)));

        can_frame candidate{};
        if (!receive(candidate, slice_us))
        {
            continue;
        }

        if ((candidate.can_id & CAN_ERR_FLAG) != 0U)
        {
            spdlog::warn("CAN error frame received on {}", _can_name);
            continue;
        }

        if (!matcher || matcher(candidate))
        {
            frame = candidate;
            return true;
        }
    }

    return false;
}

bool CANInterface::requestResponse(const can_frame& tx, can_frame& rx, const FrameMatcher& matcher, int timeout_us)
{
    if (!send(tx))
    {
        return false;
    }

    return receiveMatching(rx, matcher, timeout_us);
}

bool CANInterface::isDataAvailable(int timeout_us) const
{
    return _socket && _socket->is_data_available(timeout_us);
}

int CANInterface::socketFD() const
{
    return _socket ? _socket->get_socket_fd() : -1;
}
