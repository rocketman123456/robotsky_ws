#pragma once

#include "can/bus/can_bus_manager.h"
#include "can/can_interface.h"
#include "motor/control/motor_control.h"
#include "motor/motor_data.h"

#include <memory>
#include <mutex>
#include <vector>

enum class RobotState : uint16_t
{
    IDLE,
    RUNNING,
    ERROR,
};

struct RobotData
{
    std::mutex state_mutex;
    RobotState state = RobotState::IDLE;

    std::vector<std::shared_ptr<CANInterface>>  can_interfaces;
    std::vector<std::shared_ptr<CANBusManager>> can_buses;
    std::vector<std::shared_ptr<MotorControl>>  motors;

    std::vector<std::shared_ptr<MotorState>> motor_states;
    std::vector<std::shared_ptr<MotorCmd>>   motor_cmds;
};