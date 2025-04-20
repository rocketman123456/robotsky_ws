#include "robot/robot.h"
#include "can/can_bus_factory.h"
#include "motor/motor_factory.h"

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
        // can_interface->initialize(info);

        data->can_buses.push_back(can_bus);
    }
}

void Robot::initMotors(const std::vector<MotorInitInfo>& motor_infos)
{
    for (const auto& info : motor_infos)
    {
        auto motor = create_motor_control(info);

        data->motors.push_back(motor);
    }
}

void Robot::start()
{
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
        // rclcpp::spin_some(this);
        // 等待直到下一个时间点
        std::this_thread::sleep_until(next_time);
        next_time += interval;
    }
}

void Robot::updateFromCAN(int motorId, double pos, double vel, double torque)
{
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
    auto& motor = data->motor_cmds[motorId];

    std::lock_guard<std::mutex> lock(motor->mutex);
    motor->mode = mode;
}

void Robot::checkStateTimeouts()
{
    auto now = std::chrono::steady_clock::now();
    for (auto& motor : data->motor_states)
    {
        std::lock_guard<std::mutex> lock(motor->mutex);

        double dt = std::chrono::duration<double>(now - motor->last_rx_time).count();
        if (dt > 0.5)
        { // 0.5秒没收到反馈，算超时
            motor->health = MotorHealth::TIMEOUT;
        }
    }
}

void Robot::checkCmdTimeouts()
{
    auto now = std::chrono::steady_clock::now();
    for (auto& motor : data->motor_cmds)
    {
        std::lock_guard<std::mutex> lock(motor->mutex);

        double dt = std::chrono::duration<double>(now - motor->last_tx_time).count();
        if (dt > 0.5)
        { // 0.5秒没收到反馈，算超时
            motor->health = MotorHealth::TIMEOUT;
        }
    }
}

void Robot::tickStateMachine()
{
    checkStateTimeouts();
    // checkCmdTimeouts();

    bool hasTimeout = false;
    for (auto& motor : data->motor_states)
    {
        std::lock_guard<std::mutex> lock(motor->mutex);
        if (motor->health == MotorHealth::TIMEOUT)
        {
            hasTimeout = true;
            break;
        }
    }
    // for (auto& motor : motor_cmds)
    // {
    //     std::lock_guard<std::mutex> lock(motor->mutex);
    //     if (motor->health == MotorHealth::TIMEOUT)
    //     {
    //         hasTimeout = true;
    //         break;
    //     }
    // }

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
            // 可尝试恢复
            break;
    }
}
