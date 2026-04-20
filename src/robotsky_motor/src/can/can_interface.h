#pragma once

#include "can/socket/can_socket.h"
#include "can_data.h"

#include <linux/can.h>

#include <memory>
#include <string>

class CANInterface
{
public:
    CANInterface() = default;
    ~CANInterface();

    void initialize(const CanInitInfo& infos);
    void finalize();

    bool isInitialized() const;
    bool isDataAvailable(int timeout_us = 0) const;

    bool send(const can_frame& frame);
    bool receive(can_frame& frame);

    int getSocketFd() const;
    const std::string& getName() const;

private:
    std::string                _can_name;
    std::unique_ptr<CANSocket> _socket;
};
