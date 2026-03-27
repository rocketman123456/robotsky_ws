#pragma once

#include "robot/robot_data.h"

#include <robotsky_interface/msg/motor_cmds.hpp>
#include <robotsky_interface/msg/motor_states.hpp>

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstddef>

class Robot : public rclcpp::Node
{
public:
    Robot();
    ~Robot() = default;

    void initCAN(const std::vector<CanInitInfo>& can_infos);
    void initCANBus(const std::vector<CanBusInitInfo>& bus_infos);
    void initMotors(const std::vector<MotorInitInfo>& motor_infos);

    /** ROS interfaces for RL controller: publish /motor_states, subscribe /motor_cmds */
    void initRosInterfaces(std::size_t motor_count);

    void start();
    void stop();

    void mainLoop();

    void updateExternalCommand(int motorId, MotorMode mode);                // 外部控制接口
    void tickStateMachine();                                                // 运行状态机
    void updateFromCAN(int motorId, double pos, double vel, double torque); // CAN线程调用

    void checkStateTimeouts(); // 定期检查电机超时
    void checkCmdTimeouts();

    void publishMotorStates();

    void onMotorCmds(const robotsky_interface::msg::MotorCmds::SharedPtr msg);

    std::shared_ptr<RobotData> data;

    std::atomic<bool> running = false;

    std::size_t motor_count_{0};
    rclcpp::Publisher<robotsky_interface::msg::MotorStates>::SharedPtr motor_states_pub_;
    rclcpp::Subscription<robotsky_interface::msg::MotorCmds>::SharedPtr motor_cmds_sub_;
};