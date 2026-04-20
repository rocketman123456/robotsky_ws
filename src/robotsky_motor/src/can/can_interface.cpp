#include "can/can_interface.h"
#include "can/socket/can_socket.h"

#include <spdlog/spdlog.h>

CANInterface::~CANInterface() { finalize(); }

void CANInterface::initialize(const CanInitInfo& info)
{
    _can_name = info.can_port;
    _socket   = std::make_unique<CANSocket>();

    if (!_socket->open(info.can_port, info.enable_fd, info.rx_timeout_us))
    {
        spdlog::error("Failed to initialize CAN interface {}", _can_name);
        _socket.reset();
    }
}

void CANInterface::finalize()
{
    if (_socket)
    {
        spdlog::info("close can: {}", _can_name);
        _socket->close();
        _socket.reset();
    }
}

bool CANInterface::isInitialized() const { return _socket && _socket->isInitialized(); }

bool CANInterface::isDataAvailable(int timeout_us) const
{
    return _socket && _socket->isDataAvailable(timeout_us);
}

bool CANInterface::send(const can_frame& frame)
{
    if (!_socket)
    {
        spdlog::warn("CAN {} send skipped: socket not initialized", _can_name);
        return false;
    }

    return _socket->writeCanFrame(frame);
}

bool CANInterface::receive(can_frame& frame)
{
    if (!_socket)
    {
        spdlog::warn("CAN {} receive skipped: socket not initialized", _can_name);
        return false;
    }

    return _socket->readCanFrame(frame);
}

int CANInterface::getSocketFd() const { return _socket ? _socket->getSocketFd() : -1; }

const std::string& CANInterface::getName() const { return _can_name; }
