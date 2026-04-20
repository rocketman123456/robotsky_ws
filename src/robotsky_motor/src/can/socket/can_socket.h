#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>

#include <string>

class CANSocket
{
public:
    CANSocket() = default;
    CANSocket(const CANSocket&) = delete;
    CANSocket& operator=(const CANSocket&) = delete;

    CANSocket(CANSocket&& other) noexcept;
    CANSocket& operator=(CANSocket&& other) noexcept;

    ~CANSocket();

    bool open(const std::string& interface, bool enable_fd = false, int rx_timeout_us = 100);
    void close();

    bool writeCanFrame(const can_frame& frame) const;
    bool readCanFrame(can_frame& frame) const;
    bool isDataAvailable(int timeout_us = 0) const;

    bool isInitialized() const;
    bool isCanFdEnabled() const;
    int getSocketFd() const;
    const std::string& getInterface() const;

private:
    bool initializeSocket(const std::string& interface, bool enable_fd, int rx_timeout_us);

private:
    int         socket_fd_  = -1;
    std::string interface_  = "";
    bool        fd_enabled_ = false;
};
