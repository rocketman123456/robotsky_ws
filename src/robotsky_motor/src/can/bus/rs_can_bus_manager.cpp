#include "can/bus/rs_can_bus_manager.h"

RSCANBusManager::RSCANBusManager()
    : CANBusManager()
{
    type = CanType::RS;
}

void RSCANBusManager::step()
{
    //
}
