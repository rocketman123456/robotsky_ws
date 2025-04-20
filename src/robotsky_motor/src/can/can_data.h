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
};

struct CanInitInfo
{
    CanInitInfo(const std::string& port)
        : can_port(port)
    {}
    ~CanInitInfo() = default;

    std::string can_port;
};

struct CanBusInitInfo
{
    CanType type;

    std::vector<uint16_t> can_indices;
    std::vector<uint16_t> motor_indices;
};
