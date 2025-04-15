#pragma once

#include <string>

struct CanInitInfo
{
    CanInitInfo(const std::string& port) : can_port(port) {}
    ~CanInitInfo() = default;

    std::string can_port;
};
