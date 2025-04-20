#include "can/can_bus_factory.h"

#include "can/bus/dm_can_bus_manager.h"
#include "can/bus/rs_can_bus_manager.h"

std::shared_ptr<CANBusManager> create_can_bus_manager(CanType type)
{
    auto can_bus_manager = std::make_shared<CANBusManager>();

    switch (type)
    {
        case CanType::DM:
            return {};
        case CanType::RS:
            return {};
        case CanType::CYBER:
            return {};
        case CanType::LK:
            return {};
        default:
            return {};
    }
}