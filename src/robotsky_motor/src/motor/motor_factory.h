#pragma once

#include "motor/motor_control.h"

#include <memory>

std::shared_ptr<MotorControl> create_motor_control(const MotorInitInfo& info);
