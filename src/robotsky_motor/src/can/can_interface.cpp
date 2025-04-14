#include "can/can_interface.h"
#include "can/can_utils.h"

#include <spdlog/spdlog.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <assert.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

void CANInterface::initialize(const CanInitInfo& infos)
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

void CANInterface::send(int can_index, can_frame& frame) { write(_socket_fd, &frame, k_can_size); }

void CANInterface::receive(int can_index, can_frame& frame) { read(_socket_fd, &frame, k_can_size); }
