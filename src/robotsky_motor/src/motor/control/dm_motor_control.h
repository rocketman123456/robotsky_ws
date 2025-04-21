#pragma once

#include "motor/control/motor_control.h"

class DMMotorControl : public MotorControl
{
public:
    DMMotorControl()          = default;
    virtual ~DMMotorControl() = default;

    virtual void setZero();
    virtual void enable();
    virtual void disable();

    virtual void setMixedControlInDeg(double pos, double vel, double tau, double kp, double kd);
    virtual void setMixedControlInRad(double pos, double vel, double tau, double kp, double kd);
};
