// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <array>
#include <chrono>
#include <csignal>
#include <iostream>
#include <openarm/can/socket/openarm.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robotsky_interface/msg/motor_cmds.hpp>
#include <robotsky_interface/msg/motor_states.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <thread>
#include <vector>

namespace {

using openarm::damiao_motor::CallbackMode;
using openarm::damiao_motor::ControlMode;
using openarm::damiao_motor::MITParam;
using openarm::damiao_motor::MotorType;

constexpr std::size_t kMotorCount = 16;
constexpr double kLoopHz = 500.0;
volatile std::sig_atomic_t g_running = 1;

struct JointCommand {
    double pos = 0.0;
    double vel = 0.0;
    double tau = 0.0;
    double kp = 0.0;
    double kd = 0.0;
};

const std::array<std::string, kMotorCount> kJointNames = {
    "RF_Roll_Joint", "RF_Hip_Joint", "RF_Knee_Joint", "RF_Wheel_Joint",
    "LF_Roll_Joint", "LF_Hip_Joint", "LF_Knee_Joint", "LF_Wheel_Joint",
    "RB_Roll_Joint", "RB_Hip_Joint", "RB_Knee_Joint", "RB_Wheel_Joint",
    "LB_Roll_Joint", "LB_Hip_Joint", "LB_Knee_Joint", "LB_Wheel_Joint"};

void signal_handler(int) { g_running = 0; }

std::vector<MotorType> default_motor_types() {
    return {
        MotorType::DM4310, MotorType::DM4310, MotorType::DM4310, MotorType::DM4310,
        MotorType::DM4310, MotorType::DM4310, MotorType::DM4310, MotorType::DM4310,
        MotorType::DM4310, MotorType::DM4310, MotorType::DM4310, MotorType::DM4310,
        MotorType::DM4310, MotorType::DM4310, MotorType::DM4310, MotorType::DM4310};
}

std::vector<uint32_t> default_send_can_ids() {
    return {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
            0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10};
}

std::vector<uint32_t> default_recv_can_ids() {
    return {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
            0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20};
}

std::vector<ControlMode> default_control_modes() {
    return std::vector<ControlMode>(kMotorCount, ControlMode::MIT);
}

std::array<JointCommand, kMotorCount> stand_commands() {
    std::array<JointCommand, kMotorCount> commands{};
    for (std::size_t leg = 0; leg < 4; leg++) {
        commands[leg * 4 + 0] = JointCommand{0.0, 0.0, 0.0, 20.0, 1.0};
        commands[leg * 4 + 1] = JointCommand{0.0, 0.0, 0.0, 20.0, 1.0};
        commands[leg * 4 + 2] = JointCommand{0.0, 0.0, 0.0, 40.0, 1.0};
        commands[leg * 4 + 3] = JointCommand{0.0, 0.0, 0.0, 0.0, 0.0};
    }
    return commands;
}

void send_commands(openarm::can::socket::OpenArm& openarm,
                   const std::array<JointCommand, kMotorCount>& commands) {
    std::vector<MITParam> params;
    params.reserve(commands.size());
    for (const auto& command : commands) {
        params.push_back(MITParam{command.kp, command.kd, command.pos, command.vel, command.tau});
    }
    openarm.get_leg().mit_control_all(params);
}

void publish_states(rclcpp::Node& node, openarm::can::socket::OpenArm& openarm,
                    const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr& joint_pub,
                    const rclcpp::Publisher<robotsky_interface::msg::MotorStates>::SharedPtr&
                        motor_state_pub) {
    const auto motors = openarm.get_leg().get_motors();

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = node.now();
    joint_state.name.assign(kJointNames.begin(), kJointNames.end());
    joint_state.position.resize(kMotorCount, 0.0);
    joint_state.velocity.resize(kMotorCount, 0.0);
    joint_state.effort.resize(kMotorCount, 0.0);

    robotsky_interface::msg::MotorStates motor_states;
    motor_states.header = joint_state.header;
    motor_states.states.resize(kMotorCount);

    for (std::size_t i = 0; i < motors.size() && i < kMotorCount; i++) {
        joint_state.position[i] = motors[i].get_position();
        joint_state.velocity[i] = motors[i].get_velocity();
        joint_state.effort[i] = motors[i].get_torque();

        motor_states.states[i].pos = motors[i].get_position();
        motor_states.states[i].vel = motors[i].get_velocity();
        motor_states.states[i].tau = motors[i].get_torque();
    }

    joint_pub->publish(joint_state);
    motor_state_pub->publish(motor_states);
}

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    auto node = std::make_shared<rclcpp::Node>("openarm_wheel_legged_robot_demo");
    const std::string can_interface =
        node->declare_parameter<std::string>("can_interface", "can0");
    const bool enable_fd = node->declare_parameter<bool>("enable_fd", true);

    auto commands = stand_commands();
    auto motor_cmd_sub = node->create_subscription<robotsky_interface::msg::MotorCmds>(
        "motor_cmds", 10, [&commands](const robotsky_interface::msg::MotorCmds::SharedPtr msg) {
            if (msg->cmds.size() < kMotorCount) {
                return;
            }
            for (std::size_t i = 0; i < kMotorCount; i++) {
                commands[i].pos = msg->cmds[i].pos;
                commands[i].vel = msg->cmds[i].vel;
                commands[i].tau = msg->cmds[i].tau;
                commands[i].kp = msg->cmds[i].kp;
                commands[i].kd = msg->cmds[i].kd;
            }
        });

    auto joint_pub = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    auto motor_state_pub =
        node->create_publisher<robotsky_interface::msg::MotorStates>("motor_states", 10);

    try {
        openarm::can::socket::OpenArm openarm(can_interface, enable_fd);
        openarm.init_leg_motors(default_motor_types(), default_send_can_ids(),
                                default_recv_can_ids(), default_control_modes());
        openarm.set_callback_mode_all(CallbackMode::STATE);
        openarm.enable_all();
        openarm.recv_all(2000);

        const auto interval = std::chrono::duration<double>(1.0 / kLoopHz);
        auto next_time = std::chrono::steady_clock::now() + interval;

        while (rclcpp::ok() && g_running) {
            send_commands(openarm, commands);
            openarm.recv_all(500);
            openarm.refresh_all();
            openarm.recv_all(500);

            publish_states(*node, openarm, joint_pub, motor_state_pub);
            rclcpp::spin_some(node);

            std::this_thread::sleep_until(next_time);
            next_time += std::chrono::duration_cast<std::chrono::steady_clock::duration>(interval);
        }

        openarm.disable_all();
        openarm.recv_all(1000);
    } catch (const std::exception& e) {
        std::cerr << "openarm wheel-legged demo error: " << e.what() << std::endl;
        rclcpp::shutdown();
        return -1;
    }

    rclcpp::shutdown();
    return 0;
}
