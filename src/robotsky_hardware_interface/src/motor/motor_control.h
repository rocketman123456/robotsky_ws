#pragma once

#include "motor/motor_data.h"

#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <memory>

struct motor_init_info_t
{
    motor_init_info_t() = default;
    motor_init_info_t(uint16_t can_id_, uint16_t id_)
        : can_id(can_id_)
        , id(id_)
    {}
    motor_init_info_t(uint16_t can_id_, uint16_t id_, double dir, double off)
        : can_id(can_id_)
        , id(id_)
        , direction(dir)
        , offset(off)
    {}
    motor_init_info_t(uint16_t can_id_, uint16_t id_, double dir, double off, double delta)
        : can_id(can_id_)
        , id(id_)
        , direction(dir)
        , offset(off)
        , delta(delta)
    {}
    motor_init_info_t(uint16_t can_id_, uint16_t id_, double dir, double off, double delta, double ps, double vs, double ts)
        : can_id(can_id_)
        , id(id_)
        , direction(dir)
        , offset(off)
        , delta(delta)
        , pos_scalar(ps)
        , vel_scalar(vs)
        , tau_scalar(ts)
    {}

    uint16_t can_id     = 0;
    uint16_t id         = 0;
    double   direction  = 1.0;
    double   offset     = 0.0;
    double   delta      = 0.0;
    double   pos_scalar = 1.0;
    double   vel_scalar = 1.0;
    double   tau_scalar = 1.0;
};

// common control procedure:
//      enable -> set control parameter -> update
class MotorControl
{
public:
    MotorControl()  = default;
    ~MotorControl() = default;

    void initialize(const motor_init_info_t& info);

    void setZero();
    void enable();
    void disable();

    void setMixedControlInDeg(double pos, double vel, double tau, double kp, double kd);
    void setMixedControlInRad(double pos, double vel, double tau, double kp, double kd);

    double getPositionRad() const;
    double getVelocityRad() const;
    double getPositionDeg() const;
    double getVelocityDeg() const;
    double getTorque() const;

    // update data with infos
    void update();

    uint16_t can_id     = 0;
    uint16_t id         = 0;
    double   direction  = 1.0;
    double   offset     = 0.0;
    double   delta      = 0.0;
    double   pos_scalar = 1.0;
    double   vel_scalar = 1.0;
    double   tau_scalar = 1.0;

    double torque_upper_limit = 12.0;
    double torque_lower_limit = -12.0;

    motor_data_t data;
    can_frame    can_tx;
    can_frame    can_rx;
};
