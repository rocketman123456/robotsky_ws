#pragma once

#include "can_data.h"

#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>

#include <stdio.h>
#include <string>
#include <vector>

class CANInterface
{
public:
    CANInterface()  = default;
    ~CANInterface() = default;

    void initialize(const CanInitInfo& infos);
    void finalize();

    void send(can_frame& frame);
    void receive(can_frame& frame);

private:
    const size_t k_can_size = sizeof(struct can_frame);

    std::string _can_name;

    int _socket_fd;
};
