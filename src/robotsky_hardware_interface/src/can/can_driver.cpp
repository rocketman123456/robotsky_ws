#include "can/can_driver.h"
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

void CanDriver::initialize(const std::vector<CanInitInfo>& infos)
{
    for (auto& info : infos)
    {
        // spdlog::debug("can port: {}", info.can_port);
        auto sockfd = init_can(info.can_port);
        _sockets.push_back(sockfd);
    }
}

void CanDriver::finalize()
{
    // Close the socket
    for (auto sockfd : _sockets)
    {
        close(sockfd);
    }
}

int CanDriver::getSocket(int index)
{
    // Check if index is within bounds
    // assert(index >= 0 && index < _sockets.size());
    return _sockets[index];
}

void CanDriver::send(int can_index, can_frame& frame)
{
    assert(can_index >= 0 && can_index < _sockets.size() && "can index out of range");
    int sockfd = getSocket(can_index);
    write(sockfd, &frame, k_can_size);
}

void CanDriver::receive(int can_index, can_frame& frame)
{
    assert(can_index >= 0 && can_index < _sockets.size() && "can index out of range");
    int sockfd = getSocket(can_index);
    read(sockfd, &frame, k_can_size);
}
