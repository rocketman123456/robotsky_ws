#pragma once

#include "can/bus/can_bus_manager.h"

class DMCANBusManager : public CANBusManager
{
public:
    DMCANBusManager();
    virtual ~DMCANBusManager() = default;

protected:
    virtual void step();
};
