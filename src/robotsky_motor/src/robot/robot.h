#pragma once

#include "robot/robot_data.h"

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>

class Robot : public rclcpp::Node
{
public:
    Robot();
    ~Robot() = default;

    void initCAN(const std::vector<CanInitInfo>& can_infos);
    void initCANBus(const std::vector<CanBusInitInfo>& bus_infos);
    void initMotors(const std::vector<MotorInitInfo>& motor_infos);

    void start();
    void stop();

    void mainLoop();

    void updateExternalCommand(int motorId, MotorMode mode);                // 外部控制接口
    void tickStateMachine();                                                // 运行状态机
    void updateFromCAN(int motorId, double pos, double vel, double torque); // CAN线程调用

    void checkStateTimeouts(); // 定期检查电机超时
    void checkCmdTimeouts();

    std::shared_ptr<RobotData> data;

    std::atomic<bool> running = false;
};