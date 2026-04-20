#pragma once

#include "can_data.h"
#include "can/socket/can_socket.h"

#include <functional>
#include <linux/can.h>
#include <memory>
#include <string>

class CANInterface
{
public:
    using FrameMatcher = std::function<bool(const can_frame&)>;

    CANInterface()  = default;
    ~CANInterface() = default;

    void initialize(const CanInitInfo& infos);
    void finalize();

    bool send(const can_frame& frame);
    bool receive(can_frame& frame, int timeout_us = 1000);
    bool receiveMatching(can_frame& frame, const FrameMatcher& matcher, int timeout_us = 5000);
    bool requestResponse(const can_frame& tx, can_frame& rx, const FrameMatcher& matcher, int timeout_us = 5000);

    bool isDataAvailable(int timeout_us = 100) const;
    int  socketFD() const;

private:
    std::string _can_name;
    bool        _enable_fd = false;
    std::unique_ptr<robotsky_motor::canbus::CANSocket> _socket;
};
