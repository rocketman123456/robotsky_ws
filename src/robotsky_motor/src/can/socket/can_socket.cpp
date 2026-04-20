#include "can/socket/can_socket.h"

#include <spdlog/spdlog.h>

#include <linux/can/error.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <utility>

CANSocket::CANSocket(CANSocket&& other) noexcept
    : socket_fd_(other.socket_fd_)
    , interface_(std::move(other.interface_))
    , fd_enabled_(other.fd_enabled_)
{
    other.socket_fd_  = -1;
    other.fd_enabled_ = false;
}

CANSocket& CANSocket::operator=(CANSocket&& other) noexcept
{
    if (this == &other)
    {
        return *this;
    }

    close();

    socket_fd_  = other.socket_fd_;
    interface_  = std::move(other.interface_);
    fd_enabled_ = other.fd_enabled_;

    other.socket_fd_  = -1;
    other.fd_enabled_ = false;
    return *this;
}

CANSocket::~CANSocket() { close(); }

bool CANSocket::open(const std::string& interface, bool enable_fd, int rx_timeout_us)
{
    close();
    return initializeSocket(interface, enable_fd, rx_timeout_us);
}

void CANSocket::close()
{
    if (socket_fd_ >= 0)
    {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
}

bool CANSocket::writeCanFrame(const can_frame& frame) const
{
    if (!isInitialized())
    {
        return false;
    }

    const ssize_t bytes = ::write(socket_fd_, &frame, sizeof(frame));
    if (bytes == static_cast<ssize_t>(sizeof(frame)))
    {
        return true;
    }

    if (bytes < 0)
    {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
        {
            spdlog::warn("CAN {} write failed: {}", interface_, std::strerror(errno));
        }
        return false;
    }

    spdlog::warn("CAN {} short write: {} / {} bytes", interface_, bytes, sizeof(frame));
    return false;
}

bool CANSocket::readCanFrame(can_frame& frame) const
{
    frame = can_frame {};

    if (!isInitialized())
    {
        return false;
    }

    const ssize_t bytes = ::read(socket_fd_, &frame, sizeof(frame));
    if (bytes == static_cast<ssize_t>(sizeof(frame)))
    {
        if ((frame.can_id & CAN_ERR_FLAG) != 0U)
        {
            spdlog::warn("CAN {} error frame received: can_id=0x{:X}", interface_, frame.can_id);
            return false;
        }

        return true;
    }

    if (bytes < 0)
    {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
        {
            spdlog::warn("CAN {} read failed: {}", interface_, std::strerror(errno));
        }
        return false;
    }

    if (bytes != 0)
    {
        spdlog::warn("CAN {} short read: {} / {} bytes", interface_, bytes, sizeof(frame));
    }
    return false;
}

bool CANSocket::isDataAvailable(int timeout_us) const
{
    if (!isInitialized())
    {
        return false;
    }

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(socket_fd_, &read_fds);

    timeval timeout {};
    timeout.tv_sec  = timeout_us / 1000000;
    timeout.tv_usec = timeout_us % 1000000;

    const int result = select(socket_fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
    return result > 0 && FD_ISSET(socket_fd_, &read_fds);
}

bool CANSocket::isInitialized() const { return socket_fd_ >= 0; }

bool CANSocket::isCanFdEnabled() const { return fd_enabled_; }

int CANSocket::getSocketFd() const { return socket_fd_; }

const std::string& CANSocket::getInterface() const { return interface_; }

bool CANSocket::initializeSocket(const std::string& interface, bool enable_fd, int rx_timeout_us)
{
    interface_  = interface;
    fd_enabled_ = enable_fd;

    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0)
    {
        spdlog::error("Failed to open CAN socket for {}", interface_);
        return false;
    }

    ifreq ifr {};
    sockaddr_can addr {};

    std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0)
    {
        spdlog::error("ioctl SIOCGIFINDEX failed for {}", interface_);
        close();
        return false;
    }

    if (fd_enabled_)
    {
        int enable_canfd = 1;
        if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0)
        {
            spdlog::error("Failed to enable CAN-FD for {}", interface_);
            close();
            return false;
        }
    }

    const int flags = fcntl(socket_fd_, F_GETFL, 0);
    if (flags < 0 || fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0)
    {
        spdlog::warn("Failed to set O_NONBLOCK on CAN socket for {}", interface_);
    }

    if (rx_timeout_us > 0)
    {
        timeval timeout {};
        timeout.tv_sec  = rx_timeout_us / 1000000;
        timeout.tv_usec = rx_timeout_us % 1000000;
        if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) != 0)
        {
            spdlog::warn("Failed to set SO_RCVTIMEO on CAN socket for {}", interface_);
        }
    }

    can_err_mask_t err_mask = CAN_ERR_TX_TIMEOUT | CAN_ERR_BUSOFF;
    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask)) != 0)
    {
        spdlog::warn("setsockopt CAN_RAW_ERR_FILTER failed for {}", interface_);
    }

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
    {
        spdlog::error("Failed to bind CAN socket for {}", interface_);
        close();
        return false;
    }

    spdlog::info("open can: {}", interface_);
    return true;
}
