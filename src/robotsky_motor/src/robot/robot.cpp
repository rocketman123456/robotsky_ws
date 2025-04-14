#include "robot/robot.h"

void Robot::initCAN(std::vector<CanInitInfo>& can_infos)
{
    //
}

void Robot::initMotors(std::vector<MotorInitInfo>& motor_infos)
{
    //
}

void Robot::start()
{
    //
}

void Robot::stop()
{
    //
}

void Robot::updateFromCAN(int motorId, double pos, double vel, double torque)
{
    auto& motor = motor_states[motorId];

    std::lock_guard<std::mutex> lock(motor->mutex);
    motor->pos          = pos;
    motor->vel          = vel;
    motor->tau          = torque;
    motor->last_rx_time = std::chrono::steady_clock::now();
    motor->health       = MotorHealth::OK;
}

void Robot::updateExternalCommand(int motorId, MotorMode mode)
{
    auto& motor = motor_cmds[motorId];

    std::lock_guard<std::mutex> lock(motor->mutex);
    motor->mode = mode;
}

void Robot::checkStateTimeouts()
{
    auto now = std::chrono::steady_clock::now();
    for (auto& motor : motor_states)
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
    for (auto& motor : motor_cmds)
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
    for (auto& motor : motor_states)
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

    std::lock_guard<std::mutex> lock(state_mutex);
    switch (state)
    {
        case RobotState::IDLE:
            if (!hasTimeout)
                state = RobotState::RUNNING;
            break;
        case RobotState::RUNNING:
            if (hasTimeout)
                state = RobotState::ERROR;
            break;
        case RobotState::ERROR:
            // 可尝试恢复
            break;
    }
}
