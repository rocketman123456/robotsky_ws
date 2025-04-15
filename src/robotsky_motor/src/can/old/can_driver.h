#pragma once

#include "can/can_data.h"

#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>

#include <stdio.h>
#include <string>
#include <vector>

class CanDriver
{
public:
    CanDriver()  = default;
    ~CanDriver() = default;

    void initialize(const std::vector<CanInitInfo>& infos);
    void finalize();

    void send(int can_index, can_frame& frame);
    void receive(int can_index, can_frame& frame);

    int getSocket(int index);

private:
    const size_t k_can_size = sizeof(struct can_frame);

    std::vector<int> _sockets;
};
