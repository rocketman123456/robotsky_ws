#include "can/bus/can_bus_manager.h"
#include "robot/robot_data.h"
#include "utils/utils.h"

#include <spdlog/spdlog.h>

#include <cassert>
#include <chrono>
#include <iostream>
#include <memory>

void CANBusManager::initialize(const CanBusInitInfo& info)
{
    spdlog::info("can bus init: {}, {}", (int)info.type, info.cpu_core);

    type          = info.type;
    cpu_core      = info.cpu_core;
    can_indices   = info.can_indices;
    motor_indices = info.motor_indices;

    if(data != nullptr)
    {
        // TODO : check can type
        // check motor type
        for (auto& index : motor_indices)
        {
            assert(index >= 0 && index < data->motors.size() && "motor index out of range");
            if(data->motors[index]->type == type)
            {
                motor_index_map[data->motors[index]->id] = index;
            }
            else
            {
                spdlog::warn("motor type mismatch");
            }

            bool find = false;
            for (auto can_index : can_indices)
            {
                assert(can_index >= 0 && can_index < data->can_interfaces.size() && "can index out of range");
                if (data->motors[index]->can_index == can_index)
                {
                    find = true;
                }
            }
            if(!find)
            {
                spdlog::warn("motor can index mismatch");
            }
        }
    }
    else
    {
        spdlog::warn("cannot validate can bus config");
    }
}

void CANBusManager::setRobotData(std::shared_ptr<RobotData> data)
{
    // set data pointer
    assert(data != nullptr && "robot data pointer is null");
    this->data = data;
}

void CANBusManager::start()
{
    running       = true;
    worker_thread = std::thread(&CANBusManager::run, this);
}

void CANBusManager::stop()
{
    running = false;
    if (worker_thread.joinable())
        worker_thread.join();
}

void updateCmd(std::shared_ptr<MotorCmd> cmd)
{
    assert(cmd != nullptr && "motor cmd pointer is null");
    //
}

void updateState(std::shared_ptr<MotorState> state)
{
    assert(state != nullptr && "motor state pointer is null");
    //
}

void CANBusManager::step()
{
    // TODO
}

void CANBusManager::run()
{
    set_thread(cpu_core, worker_thread.native_handle());

    counter.print_info = false;
    counter.start();

    while (running.load())
    {
        step();

        counter.update();

        // 等待直到下一个时间点
        std::this_thread::sleep_until(next_time);
        next_time += interval;
    }
}
