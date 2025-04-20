#pragma once

#include "motor/control/motor_control.h"

#include <memory>

std::shared_ptr<MotorControl> create_motor_control(const MotorInitInfo& info);
