#pragma once

#include "motor/motor_control.h"

class DMMotorControl : public MotorControl
{
public:
    DMMotorControl()          = default;
    virtual ~DMMotorControl() = default;

    void initialize(const MotorInitInfo& info);

    virtual void setZero();
    virtual void enable();
    virtual void disable();

    virtual void setMixedControlInDeg(double pos, double vel, double tau, double kp, double kd);
    virtual void setMixedControlInRad(double pos, double vel, double tau, double kp, double kd);
};
