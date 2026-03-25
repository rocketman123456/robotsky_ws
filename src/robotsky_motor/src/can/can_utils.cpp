#include "can/can_utils.h"

#include <spdlog/spdlog.h>

#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <fcntl.h>
#include <cstring>
#include <unistd.h>

int init_can(const std::string& port)
{
    ifreq        ifr{};
    sockaddr_can addr{};

    int sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sockfd < 0)
    {
        spdlog::error("Error while opening CAN socket for {}", port);
        return -1;
    }

    std::strncpy(ifr.ifr_name, port.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(sockfd, SIOCGIFINDEX, &ifr) < 0)
    {
        spdlog::error("ioctl SIOCGIFINDEX failed for {}", port);
        close(sockfd);
        return -1;
    }

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sockfd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0)
    {
        spdlog::error("Error binding CAN socket for {}", port);
        close(sockfd);
        return -1;
    }

    int flags = fcntl(sockfd, F_GETFL, 0);
    if (flags < 0 || fcntl(sockfd, F_SETFL, flags | O_NONBLOCK) < 0)
    {
        spdlog::warn("Failed to set O_NONBLOCK on CAN socket for {}", port);
    }

    can_err_mask_t err_mask = CAN_ERR_TX_TIMEOUT | CAN_ERR_BUSOFF;
    if (setsockopt(sockfd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask)) != 0)
    {
        spdlog::warn("setsockopt CAN_RAW_ERR_FILTER failed for {}", port);
    }
    else
    {
        spdlog::debug("CAN {} init done", port);
    }

    return sockfd;
}
