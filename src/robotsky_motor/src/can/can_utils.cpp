#include "can/can_utils.h"

#include <spdlog/spdlog.h>

#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>

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

int init_can(const std::string& port)
{
    ifreq        ifr;
    sockaddr_can addr;

    int setflag = 0;
    int ret0    = 0;
    int sockfd  = 0;

    // open the socket
    if ((sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        spdlog::error("Error while opening socket");
    }

    strcpy(ifr.ifr_name, port.c_str());
    ioctl(sockfd, SIOCGIFINDEX, &ifr);
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
        spdlog::error("Error in socket bind");
    }

    setflag = setflag | O_NONBLOCK;
    ret0    = fcntl(sockfd, F_SETFL, setflag);
    fcntl(sockfd, F_GETFL, 0);

    can_err_mask_t err_mask = CAN_ERR_TX_TIMEOUT | CAN_ERR_BUSOFF;
    if (setsockopt(sockfd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask)) != 0)
    {
        spdlog::warn("Error in setsockopt");
    }
    else
    {
        spdlog::debug("CAN {} Init Done", port);
    }

    return sockfd;
}
