#include "can/bus/dm_can_bus_manager.h"

DMCANBusManager::DMCANBusManager()
    : CANBusManager()
{
    type = CanType::DM;
}

void DMCANBusManager::step()
{
    //
}
