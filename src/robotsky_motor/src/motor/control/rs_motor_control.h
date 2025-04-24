#pragma once

#include "motor/control/motor_control.h"

class RSMotorControl : public MotorControl
{
public:
    RSMotorControl()          = default;
    virtual ~RSMotorControl() = default;

    virtual void setZero();
    virtual void enable();
    virtual void disable();

    // void setMITMode();

    virtual void setMixedControlInDeg(double pos, double vel, double tau, double kp, double kd);
    virtual void setMixedControlInRad(double pos, double vel, double tau, double kp, double kd);
};
