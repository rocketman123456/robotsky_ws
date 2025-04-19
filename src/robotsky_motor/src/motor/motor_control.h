#pragma once

#include "motor/motor_data.h"

#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <memory>

// common control procedure:
//   enable -> set control parameter -> update
class MotorControl
{
public:
    MotorControl()          = default;
    virtual ~MotorControl() = default;

    virtual void initialize(const MotorInitInfo& info);

    virtual void setZero() = 0;
    virtual void enable()  = 0;
    virtual void disable() = 0;

    virtual void setMixedControlInDeg(double pos, double vel, double tau, double kp, double kd) = 0;
    virtual void setMixedControlInRad(double pos, double vel, double tau, double kp, double kd) = 0;

    // static virtual MotorState decode() = 0;

    virtual void update();

    double getPositionRad() const;
    double getVelocityRad() const;
    double getPositionDeg() const;
    double getVelocityDeg() const;
    double getTorque() const;

protected:
    void forwardDataComputeDeg(double pos, double vel, double tau, double kp, double kd);
    void forwardDataComputeRad(double pos, double vel, double tau, double kp, double kd);

public:
    uint16_t can_index  = 0;
    uint16_t id         = 0;
    double   direction  = 1.0;
    double   offset     = 0.0;
    double   delta      = 0.0;
    double   pos_scalar = 1.0;
    double   vel_scalar = 1.0;
    double   tau_scalar = 1.0;

    double torque_upper_limit = 12.0;
    double torque_lower_limit = -12.0;

    MotorState state;
    MotorCmd   cmd;

    can_frame can_tx;
    can_frame can_rx;
};
