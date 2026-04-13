#include "can/can_interface.h"
#include "can/can_utils.h"

#include <spdlog/spdlog.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <assert.h>
#include <cerrno>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <string.h>
#include <unistd.h>

void CANInterface::initialize(const CanInitInfo& info)
{
    _can_name  = info.can_port;
    _socket_fd = init_can(info.can_port);
    spdlog::info("open can: {}", _can_name);
}

void CANInterface::finalize()
{
    spdlog::info("close can: {}", _can_name);
    close(_socket_fd);
}

bool CANInterface::send(const can_frame& frame)
{
    if (_socket_fd < 0)
    {
        spdlog::warn("CAN {} send skipped: socket not initialized", _can_name);
        return false;
    }

    const ssize_t bytes = write(_socket_fd, &frame, k_can_size);
    if (bytes == static_cast<ssize_t>(k_can_size))
    {
        return true;
    }

    if (bytes < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            spdlog::debug("CAN {} send would block", _can_name);
        }
        else
        {
            spdlog::warn("CAN {} send failed: {}", _can_name, std::strerror(errno));
        }
        return false;
    }

    spdlog::warn("CAN {} short write: {} / {} bytes", _can_name, bytes, k_can_size);
    return false;
}

bool CANInterface::receive(can_frame& frame)
{
    frame = can_frame{};

    if (_socket_fd < 0)
    {
        spdlog::warn("CAN {} receive skipped: socket not initialized", _can_name);
        return false;
    }

    const ssize_t bytes = read(_socket_fd, &frame, k_can_size);
    if (bytes == static_cast<ssize_t>(k_can_size))
    {
        if ((frame.can_id & CAN_ERR_FLAG) != 0U)
        {
            spdlog::warn("CAN {} error frame received: can_id=0x{:X}", _can_name, frame.can_id);
            return false;
        }
        return true;
    }

    if (bytes < 0)
    {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
        {
            spdlog::warn("CAN {} receive failed: {}", _can_name, std::strerror(errno));
        }
        return false;
    }

    if (bytes != 0)
    {
        spdlog::warn("CAN {} short read: {} / {} bytes", _can_name, bytes, k_can_size);
    }
    return false;
}
