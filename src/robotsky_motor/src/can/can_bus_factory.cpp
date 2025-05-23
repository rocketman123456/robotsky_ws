#include "can/can_bus_factory.h"

#include "can/bus/dm_can_bus_manager.h"
#include "can/bus/rs_can_bus_manager.h"

std::shared_ptr<CANBusManager> create_can_bus_manager(CanType type)
{
    switch (type)
    {
        case CanType::DM:
            return std::make_shared<DMCANBusManager>();
        case CanType::RS:
            return std::make_shared<RSCANBusManager>();
        case CanType::CYBER:
            return {};
        case CanType::LK:
            return {};
        default:
            return {};
    }
}