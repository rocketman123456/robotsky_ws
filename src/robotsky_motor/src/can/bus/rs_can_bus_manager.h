#pragma once

#include "can/bus/can_bus_manager.h"

class RSCANBusManager : public CANBusManager
{
public:
    RSCANBusManager();
    virtual ~RSCANBusManager() = default;

protected:
    virtual void step();
};