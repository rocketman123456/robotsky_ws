#pragma once
#include "motor/motor_control.h"

class CyberMotorControl : public MotorControl
{
public:
    CyberMotorControl()          = default;
    virtual ~CyberMotorControl() = default;

    virtual void setZero();
    virtual void enable();
    virtual void disable();

    virtual void setMixedControlInDeg(double pos, double vel, double tau, double kp, double kd);
    virtual void setMixedControlInRad(double pos, double vel, double tau, double kp, double kd);
};
