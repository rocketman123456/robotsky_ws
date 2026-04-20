#include "robot/robot.h"
#include "can/can_bus_factory.h"
#include "motor/motor_factory.h"

#include <algorithm>
#include <functional>

#include <spdlog/spdlog.h>

namespace
{
std::size_t getMotorSlotCount(const std::vector<MotorInitInfo>& motor_infos)
{
    std::size_t slot_count = 0;
    for (const auto& info : motor_infos)
    {
        slot_count = std::max(slot_count, static_cast<std::size_t>(info.id));
    }
    return slot_count;
}
} // namespace

Robot::Robot()
    : Node("robotsky_motor")
{
    data = std::make_shared<RobotData>();
}

void Robot::initCAN(const std::vector<CanInitInfo>& can_infos)
{
    for (const auto& info : can_infos)
    {
        auto can_interface = std::make_shared<CANInterface>();
        can_interface->initialize(info);

        data->can_interfaces.push_back(can_interface);
    }
}

void Robot::initCANBus(const std::vector<CanBusInitInfo>& bus_infos)
{
    for (const auto& info : bus_infos)
    {
        auto can_bus = create_can_bus_manager(info.type);
        can_bus->setRobotData(data);
        can_bus->initialize(info);

        data->can_buses.push_back(can_bus);
    }
}

void Robot::initMotors(const std::vector<MotorInitInfo>& motor_infos)
{
    const auto        now        = std::chrono::steady_clock::now();
    const std::size_t slot_count = getMotorSlotCount(motor_infos);

    if (data->motor_states.size() < slot_count)
    {
        data->motor_states.resize(slot_count);
    }
    if (data->motor_cmds.size() < slot_count)
    {
        data->motor_cmds.resize(slot_count);
    }

    for (const auto& info : motor_infos)
    {
        if (info.id == 0)
        {
            spdlog::warn("Skip motor with invalid id 0 on can{}", info.can_index);
            continue;
        }

        const std::size_t slot = static_cast<std::size_t>(info.id - 1);

        auto motor_state = std::make_shared<MotorState>();
        motor_state->mode         = info.mode;
        motor_state->last_rx_time = now;
        data->motor_states[slot]  = motor_state;

        auto motor_cmd = std::make_shared<MotorCmd>();
        motor_cmd->mode         = info.mode;
        motor_cmd->last_tx_time = now;
        data->motor_cmds[slot]  = motor_cmd;
    }

    data->motors.reserve(data->motors.size() + motor_infos.size());
    for (const auto& info : motor_infos)
    {
        auto motor = create_motor_control(info);
        motor->initialize(info);
        data->motors.push_back(motor);
    }
}

void Robot::initRosInterfaces(std::size_t motor_count)
{
    motor_count_ = motor_count;

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    motor_states_pub_ = create_publisher<robotsky_interface::msg::MotorStates>(
        "motor_states",
        qos
    );
    motor_cmds_sub_   = create_subscription<robotsky_interface::msg::MotorCmds>(
        "motor_cmds",
        qos,
        std::bind(&Robot::onMotorCmds, this, std::placeholders::_1)
    );
}

void Robot::onMotorCmds(const robotsky_interface::msg::MotorCmds::SharedPtr msg)
{
    if (motor_count_ == 0 || data->motor_cmds.size() < motor_count_)
    {
        return;
    }

    const std::size_t n   = std::min<std::size_t>(motor_count_, msg->cmds.size());
    const auto        now = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < n; ++i)
    {
        auto& mc = data->motor_cmds[i];
        if (!mc)
        {
            continue;
        }

        mc->pos          = msg->cmds[i].pos;
        mc->vel          = msg->cmds[i].vel;
        mc->tau          = msg->cmds[i].tau;
        mc->kp           = msg->cmds[i].kp;
        mc->kd           = msg->cmds[i].kd;
        mc->last_tx_time = now;
        mc->health       = MotorHealth::OK;
    }
}

void Robot::publishMotorStates()
{
    if (motor_count_ == 0 || !motor_states_pub_ || data->motor_states.size() < motor_count_)
    {
        return;
    }

    robotsky_interface::msg::MotorStates out;
    out.header.stamp = now();
    out.states.resize(motor_count_);

    for (std::size_t i = 0; i < motor_count_; ++i)
    {
        if (!data->motor_states[i])
        {
            continue;
        }

        std::lock_guard<std::mutex> lock(data->motor_states[i]->mutex);
        out.states[i].pos = data->motor_states[i]->pos;
        out.states[i].vel = data->motor_states[i]->vel;
        out.states[i].tau = data->motor_states[i]->tau;
    }

    motor_states_pub_->publish(out);
}

void Robot::start()
{
    for (auto& can_bus : data->can_buses)
    {
        can_bus->enable();
    }

    for (auto& can_bus : data->can_buses)
    {
        can_bus->start();
    }
}

void Robot::stop()
{
    for (auto& can_bus : data->can_buses)
    {
        can_bus->stop();
    }
}

void Robot::mainLoop()
{
    using namespace std::chrono;
    double frequency_hz = 500;
    auto   interval     = duration<double>(1.0 / frequency_hz);
    auto   next_time    = steady_clock::now() + interval;

    while (running)
    {
        std::this_thread::sleep_until(next_time);
        next_time += interval;
    }
}

void Robot::updateFromCAN(int motorId, double pos, double vel, double torque)
{
    if (motorId < 0 || static_cast<std::size_t>(motorId) >= data->motor_states.size() || !data->motor_states[motorId])
    {
        spdlog::error("motorId out of range: {}", motorId);
        return;
    }

    auto& motor = data->motor_states[motorId];

    std::lock_guard<std::mutex> lock(motor->mutex);
    motor->pos          = pos;
    motor->vel          = vel;
    motor->tau          = torque;
    motor->last_rx_time = std::chrono::steady_clock::now();
    motor->health       = MotorHealth::OK;
}

void Robot::updateExternalCommand(int motorId, MotorMode mode)
{
    if (motorId < 0 || static_cast<std::size_t>(motorId) >= data->motor_cmds.size() || !data->motor_cmds[motorId])
    {
        spdlog::error("motorId out of range: {}", motorId);
        return;
    }

    auto& motor = data->motor_cmds[motorId];

    std::lock_guard<std::mutex> lock(motor->mutex);
    motor->mode = mode;
}

void Robot::checkStateTimeouts()
{
    auto now = std::chrono::steady_clock::now();
    for (auto& motor : data->motor_states)
    {
        if (!motor)
        {
            continue;
        }

        std::lock_guard<std::mutex> lock(motor->mutex);

        const double dt = std::chrono::duration<double>(now - motor->last_rx_time).count();
        if (dt > 0.5)
        {
            motor->health = MotorHealth::TIMEOUT;
        }
    }
}

void Robot::checkCmdTimeouts()
{
    auto now = std::chrono::steady_clock::now();
    for (auto& motor : data->motor_cmds)
    {
        if (!motor)
        {
            continue;
        }

        std::lock_guard<std::mutex> lock(motor->mutex);

        const double dt = std::chrono::duration<double>(now - motor->last_tx_time).count();
        if (dt > 0.5)
        {
            motor->health = MotorHealth::TIMEOUT;
        }
    }
}

void Robot::tickStateMachine()
{
    checkStateTimeouts();

    bool hasTimeout = false;
    for (auto& motor : data->motor_states)
    {
        if (!motor)
        {
            continue;
        }

        std::lock_guard<std::mutex> lock(motor->mutex);
        if (motor->health == MotorHealth::TIMEOUT)
        {
            hasTimeout = true;
            break;
        }
    }

    std::lock_guard<std::mutex> lock(data->state_mutex);
    switch (data->state)
    {
        case RobotState::IDLE:
            if (!hasTimeout)
                data->state = RobotState::RUNNING;
            break;
        case RobotState::RUNNING:
            if (hasTimeout)
                data->state = RobotState::ERROR;
            break;
        case RobotState::ERROR:
            break;
    }

    publishMotorStates();
}
