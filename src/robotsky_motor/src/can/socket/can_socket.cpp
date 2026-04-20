#include "can/socket/can_socket.h"

#include <spdlog/spdlog.h>

#include <errno.h>
#include <linux/can/error.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>

namespace robotsky_motor::canbus
{

CANSocket::CANSocket(const std::string& interface, bool enable_fd)
    : interface_(interface),
      fd_enabled_(enable_fd)
{
    if (!initialize_socket(interface))
    {
        throw CANSocketException("Failed to initialize socket for interface: " + interface);
    }
}

CANSocket::~CANSocket() { cleanup(); }

bool CANSocket::initialize_socket(const std::string& interface)
{
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0)
    {
        spdlog::error("Error while opening CAN socket for {}", interface);
        return false;
    }

    ifreq        ifr{};
    sockaddr_can addr{};

    std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0)
    {
        spdlog::error("ioctl SIOCGIFINDEX failed for {}", interface);
        cleanup();
        return false;
    }

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (fd_enabled_)
    {
        int enable_canfd = 1;
        if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0)
        {
            spdlog::error("setsockopt CAN_RAW_FD_FRAMES failed for {}", interface);
            cleanup();
            return false;
        }
    }

    if (bind(socket_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
    {
        spdlog::error("Error binding CAN socket for {}", interface);
        cleanup();
        return false;
    }

    timeval timeout{};
    timeout.tv_sec  = 0;
    timeout.tv_usec = 100;
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0)
    {
        spdlog::warn("setsockopt SO_RCVTIMEO failed for {}", interface);
    }

    can_err_mask_t err_mask = CAN_ERR_TX_TIMEOUT | CAN_ERR_BUSOFF;
    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask)) != 0)
    {
        spdlog::warn("setsockopt CAN_RAW_ERR_FILTER failed for {}", interface);
    }

    return true;
}

void CANSocket::cleanup()
{
    if (socket_fd_ >= 0)
    {
        close(socket_fd_);
        socket_fd_ = -1;
    }
}

ssize_t CANSocket::read_raw_frame(void* buffer, size_t buffer_size)
{
    if (!is_initialized())
    {
        return -1;
    }

    return read(socket_fd_, buffer, buffer_size);
}

ssize_t CANSocket::write_raw_frame(const void* buffer, size_t frame_size)
{
    if (!is_initialized())
    {
        return -1;
    }

    return write(socket_fd_, buffer, frame_size);
}

bool CANSocket::write_can_frame(const can_frame& frame)
{
    return write_raw_frame(&frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
}

bool CANSocket::write_canfd_frame(const canfd_frame& frame)
{
    return write_raw_frame(&frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
}

bool CANSocket::read_can_frame(can_frame& frame)
{
    if (!is_initialized())
    {
        return false;
    }

    const ssize_t bytes_read = read_raw_frame(&frame, sizeof(frame));
    if (bytes_read == static_cast<ssize_t>(sizeof(frame)))
    {
        return true;
    }

    if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
    {
        spdlog::debug("Failed to read CAN frame from {}: {}", interface_, strerror(errno));
    }

    return false;
}

bool CANSocket::read_canfd_frame(canfd_frame& frame)
{
    if (!is_initialized())
    {
        return false;
    }

    const ssize_t bytes_read = read_raw_frame(&frame, sizeof(frame));
    if (bytes_read == static_cast<ssize_t>(sizeof(frame)))
    {
        return true;
    }

    if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
    {
        spdlog::debug("Failed to read CAN FD frame from {}: {}", interface_, strerror(errno));
    }

    return false;
}

bool CANSocket::is_data_available(int timeout_us) const
{
    if (!is_initialized())
    {
        return false;
    }

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(socket_fd_, &read_fds);

    timeval timeout{};
    timeout.tv_sec  = timeout_us / 1000000;
    timeout.tv_usec = timeout_us % 1000000;

    const int result = select(socket_fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
    return result > 0 && FD_ISSET(socket_fd_, &read_fds);
}

bool CANSocket::set_filters(const std::vector<can_filter>& filters)
{
    if (!is_initialized())
    {
        return false;
    }

    const auto* filter_ptr  = filters.empty() ? nullptr : filters.data();
    const auto  filter_size = static_cast<socklen_t>(filters.size() * sizeof(can_filter));

    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filter_ptr, filter_size) < 0)
    {
        spdlog::warn("setsockopt CAN_RAW_FILTER failed for {}", interface_);
        return false;
    }

    return true;
}

} // namespace robotsky_motor::canbus
