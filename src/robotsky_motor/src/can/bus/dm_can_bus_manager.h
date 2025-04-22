#pragma once

#include "can/bus/can_bus_manager.h"

class DMCANBusManager : public CANBusManager
{
public:
    DMCANBusManager();
    virtual ~DMCANBusManager() = default;

    virtual void enable();
    virtual void disable();
    virtual void step();
};
