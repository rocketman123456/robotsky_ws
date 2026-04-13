#pragma once

#include "can_data.h"

#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>

#include <cstddef>
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

    bool send(const can_frame& frame);
    bool receive(can_frame& frame);

private:
    const size_t k_can_size = sizeof(struct can_frame);

    std::string _can_name;

    int _socket_fd;
};
