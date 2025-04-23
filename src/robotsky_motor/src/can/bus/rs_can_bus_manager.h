#pragma once
#include "can/bus/can_bus_manager.h"
#include "motor/utils/rs_motor_utils.h"

class RSCANBusManager : public CANBusManager
{
public:
    RSCANBusManager();
    virtual ~RSCANBusManager() = default;

    virtual void enable();
    virtual void disable();
    virtual void step();

private:
    void writeState(uint16_t index, const rs_motor_fb_t& data_fb, const rs_data_read_write& data_motor);
};