#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>

#include <cstddef>
#include <stdexcept>
#include <string>
#include <sys/types.h>
#include <vector>

namespace robotsky_motor::canbus
{

class CANSocketException : public std::runtime_error
{
public:
    explicit CANSocketException(const std::string& message)
        : std::runtime_error("Socket error: " + message)
    {
    }
};

class CANSocket
{
public:
    explicit CANSocket(const std::string& interface, bool enable_fd = false);
    ~CANSocket();

    CANSocket(const CANSocket&)            = delete;
    CANSocket& operator=(const CANSocket&) = delete;
    CANSocket(CANSocket&&)                 = default;
    CANSocket& operator=(CANSocket&&)      = default;

    int                get_socket_fd() const { return socket_fd_; }
    const std::string& get_interface() const { return interface_; }
    bool               is_canfd_enabled() const { return fd_enabled_; }
    bool               is_initialized() const { return socket_fd_ >= 0; }

    ssize_t read_raw_frame(void* buffer, size_t buffer_size);
    ssize_t write_raw_frame(const void* buffer, size_t frame_size);

    bool write_can_frame(const can_frame& frame);
    bool write_canfd_frame(const canfd_frame& frame);

    bool read_can_frame(can_frame& frame);
    bool read_canfd_frame(canfd_frame& frame);

    bool is_data_available(int timeout_us = 100) const;
    bool set_filters(const std::vector<can_filter>& filters);

private:
    bool initialize_socket(const std::string& interface);
    void cleanup();

    int         socket_fd_ = -1;
    std::string interface_;
    bool        fd_enabled_ = false;
};

} // namespace robotsky_motor::canbus
