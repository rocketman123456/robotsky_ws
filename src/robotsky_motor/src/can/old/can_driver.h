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
    using FrameMatcher = CANInterface::FrameMatcher;

    CanDriver()  = default;
    ~CanDriver() = default;

    void initialize(const std::vector<CanInitInfo>& infos);
    void finalize();

    bool send(int can_index, const can_frame& frame);
    bool receive(int can_index, can_frame& frame, int timeout_us = 1000);
    bool receiveMatching(int can_index, can_frame& frame, const FrameMatcher& matcher, int timeout_us = 5000);
    bool requestResponse(int can_index, const can_frame& tx, can_frame& rx, const FrameMatcher& matcher, int timeout_us = 5000);

    int getSocket(int index);

private:
    std::vector<std::unique_ptr<CANInterface>> _interfaces;
};
