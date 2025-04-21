#pragma once

#include "can/bus/can_bus_manager.h"
#include "can/can_data.h"

#include <memory>

std::shared_ptr<CANBusManager> create_can_bus_manager(CanType type);
