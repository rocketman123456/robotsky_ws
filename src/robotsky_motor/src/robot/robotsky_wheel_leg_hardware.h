#pragma once

#include "can/can_data.h"
#include "motor/motor_data.h"

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace robotsky
{
namespace wheel_leg
{
constexpr std::size_t kMotorCount = 16;

struct StartupTargets
{
    std::array<float, kMotorCount> pos_begin;
    std::array<float, kMotorCount> pos_hold;
    std::array<float, kMotorCount> vel;
    std::array<float, kMotorCount> tau;
    std::array<float, kMotorCount> kp_begin;
    std::array<float, kMotorCount> kp_hold;
    std::array<float, kMotorCount> kd;
};

std::vector<CanInitInfo> prepareCan();
std::vector<MotorInitInfo> prepareMotors();
std::vector<CanBusInitInfo> prepareCanBuses();
std::vector<std::string> prepareJointNames();
StartupTargets prepareStartupTargets();

} // namespace wheel_leg
} // namespace robotsky
