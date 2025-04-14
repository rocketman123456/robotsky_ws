#pragma once

#include <string>

struct CanInitInfo
{
    CanInitInfo(const std::string& port)
        : can_port(port)
    {}

    std::string can_port;
};
