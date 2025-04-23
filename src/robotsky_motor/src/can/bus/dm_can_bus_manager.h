#pragma once
#include "can/bus/can_bus_manager.h"
#include "motor/utils/dm_motor_utils.h"

class DMCANBusManager : public CANBusManager
{
public:
    DMCANBusManager();
    virtual ~DMCANBusManager() = default;

    virtual void enable();
    virtual void disable();
    virtual void step();

private:
    void writeState(const dm_motor_fb_t& data_fb);
};
