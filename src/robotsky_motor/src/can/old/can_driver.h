#pragma once

#include "can/can_data.h"
#include "can/can_interface.h"

#include <linux/can.h>

#include <memory>
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
    std::vector<std::shared_ptr<CANInterface>> _interfaces;
};
