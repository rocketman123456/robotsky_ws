#pragma once

#include "motor/motor_data.h"

#include <cstdint>
#include <string>
#include <vector>

using CanType = MotorType;

struct CanInitInfo
{
    std::string can_port = "";
};

struct CanBusInitInfo
{
    CanType type = CanType::NONE;

    uint16_t cpu_core = 0;

    std::vector<uint16_t> can_indices   = {};
    std::vector<uint16_t> motor_indices = {};
};
