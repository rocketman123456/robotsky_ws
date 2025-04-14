#pragma once

#include <rclcpp/rclcpp.hpp>

#include "can/can_bus_manager.h"
#include "can/can_interface.h"
#include "motor/motor_data.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

enum class RobotState : uint16_t
{
    IDLE,
    RUNNING,
    ERROR
};

class Robot : public rclcpp::Node
{
public:
    Robot();
    ~Robot() = default;

    void initCAN(const std::vector<CanInitInfo>& can_infos);
    void initMotors(const std::vector<MotorInitInfo>& motor_infos);

    void start();
    void stop();

    void mainLoop();

    void updateExternalCommand(int motorId, MotorMode mode);                // 外部控制接口
    void tickStateMachine();                                                // 运行状态机
    void updateFromCAN(int motorId, double pos, double vel, double torque); // CAN线程调用

    void checkStateTimeouts(); // 定期检查电机超时
    void checkCmdTimeouts();

    std::vector<std::shared_ptr<MotorState>> motor_states;
    std::vector<std::shared_ptr<MotorCmd>>   motor_cmds;

    std::vector<std::shared_ptr<CANInterface>>  can_interfaces;
    std::vector<std::shared_ptr<CANBusManager>> can_buses;

    // 状态机状态（简化）
    RobotState state = RobotState::IDLE;

    std::atomic<bool> running = false;

    std::mutex state_mutex;
};