#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>

namespace robotsky_motor::canbus
{

class CANDevice
{
public:
    explicit CANDevice(canid_t send_can_id, canid_t recv_can_id, canid_t recv_can_mask, bool is_fd_enabled = false)
        : send_can_id_(send_can_id),
          recv_can_id_(recv_can_id),
          recv_can_mask_(recv_can_mask),
          is_fd_enabled_(is_fd_enabled)
    {
    }

    virtual ~CANDevice() = default;

    virtual void callback(const can_frame& frame)   = 0;
    virtual void callback(const canfd_frame& frame) = 0;

    canid_t get_send_can_id() const { return send_can_id_; }
    canid_t get_recv_can_id() const { return recv_can_id_; }
    canid_t get_recv_can_mask() const { return recv_can_mask_; }
    bool    is_fd_enabled() const { return is_fd_enabled_; }

protected:
    canid_t send_can_id_;
    canid_t recv_can_id_;
    canid_t recv_can_mask_ = CAN_SFF_MASK;
    bool    is_fd_enabled_ = false;
};

} // namespace robotsky_motor::canbus
