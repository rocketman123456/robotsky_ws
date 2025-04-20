#pragma once

#include <cstdint>
#include <string>
#include <vector>

enum class CanType : uint32_t
{
    DM,
    RS,
    CYBER,
    LK,
    NONE,
};

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
