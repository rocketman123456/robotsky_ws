#include "can/can_bus_factory.h"
#include "can/can_data.h"
#include "motor/motor_data.h"
#include "motor/motor_factory.h"
#include "robot/robot.h"
#include "robot/robotsky_wheel_leg_hardware.h"
#include "utils/fps_counter.h"
#include "utils/utils.h"

#include <rclcpp/rclcpp.hpp>

#include <robotsky_interface/msg/motor_cmds.hpp>
#include <robotsky_interface/msg/motor_states.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <spdlog/spdlog.h>

#include <chrono>
#include <memory>
#include <thread>

using namespace std::chrono_literals;
using Clock    = std::chrono::high_resolution_clock;
using Duration = std::chrono::duration<double>;

sensor_msgs::msg::JointState         joint_rviz_state;
robotsky_interface::msg::MotorCmds   motor_cmds;
robotsky_interface::msg::MotorStates motor_states;

std::shared_ptr<RobotData> data = std::make_shared<RobotData>();

constexpr std::size_t motor_count = robotsky::wheel_leg::kMotorCount;

void prepare_hardware()
{
    const auto can_infos     = robotsky::wheel_leg::prepareCan();
    const auto motor_infos   = robotsky::wheel_leg::prepareMotors();
    const auto can_bus_infos = robotsky::wheel_leg::prepareCanBuses();

    data->motor_states.resize(motor_count);
    data->motor_cmds.resize(motor_count);
    for (std::size_t i = 0; i < motor_count; ++i)
    {
        data->motor_states[i] = std::make_shared<MotorState>();
        data->motor_cmds[i]   = std::make_shared<MotorCmd>();
    }

    for (const auto& info : can_infos)
    {
        auto can_interface = std::make_shared<CANInterface>();
        can_interface->initialize(info);
        data->can_interfaces.push_back(can_interface);
    }

    for (const auto& info : motor_infos)
    {
        auto motor = create_motor_control(info);
        motor->initialize(info);
        data->motors.push_back(motor);
    }

    for (const auto& info : can_bus_infos)
    {
        auto can_bus = create_can_bus_manager(info.type);
        can_bus->setRobotData(data);
        can_bus->initialize(info);
        data->can_buses.push_back(can_bus);
    }
}

void motor_cmd_callback(const robotsky_interface::msg::MotorCmds::SharedPtr cmd)
{
    if (cmd->cmds.size() < motor_count)
    {
        spdlog::error("cmd->cmds.size() < motor_count");
        return;
    }

    for (std::size_t i = 0; i < motor_count; ++i)
    {
        data->motor_cmds[i]->pos = cmd->cmds[i].pos;
        data->motor_cmds[i]->vel = cmd->cmds[i].vel;
        data->motor_cmds[i]->tau = cmd->cmds[i].tau;
        data->motor_cmds[i]->kp  = cmd->cmds[i].kp;
        data->motor_cmds[i]->kd  = cmd->cmds[i].kd;
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto robot = std::make_shared<Robot>();

    auto joint_rviz_pub  = robot->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    auto joint_state_pub = robot->create_publisher<robotsky_interface::msg::MotorStates>("motor_states", 10);

    joint_rviz_state.position.resize(motor_count);
    joint_rviz_state.velocity.resize(motor_count);
    joint_rviz_state.effort.resize(motor_count);
    joint_rviz_state.name = robotsky::wheel_leg::prepareJointNames();

    motor_cmds.cmds.resize(motor_count);
    motor_states.states.resize(motor_count);

    prepare_hardware();

    auto startup   = robotsky::wheel_leg::prepareStartupTargets();
    auto pos       = startup.pos_begin;
    auto pos_target = startup.pos_begin;
    auto pos_end   = startup.pos_hold;
    auto vel       = startup.vel;
    auto tau       = startup.tau;
    auto kp        = startup.kp_begin;
    auto kp_target = startup.kp_begin;
    auto kp_end    = startup.kp_hold;
    auto kd        = startup.kd;

    for (std::size_t i = 0; i < motor_count; ++i)
    {
        data->motor_cmds[i]->pos = pos[i];
        data->motor_cmds[i]->vel = vel[i];
        data->motor_cmds[i]->tau = tau[i];
        data->motor_cmds[i]->kp  = kp[i];
        data->motor_cmds[i]->kd  = kd[i];
    }

    for (auto can_bus : data->can_buses)
    {
        can_bus->enable();
    }
    for (auto can_bus : data->can_buses)
    {
        can_bus->enable();
    }

    for (auto can_bus : data->can_buses)
    {
        can_bus->start();
    }

    const double frequency_hz = 500.0;
    const auto   interval     = Duration(1.0 / frequency_hz);
    auto         next_time    = Clock::now() + interval;

    auto thread_id     = std::this_thread::get_id();
    auto native_handle = *reinterpret_cast<std::thread::native_handle_type*>(&thread_id);
    set_thread(0, native_handle);

    FPSCounter fps_counter(true, "motor_control_node");
    fps_counter.start();

    float dt         = 0.0f;
    float total_time = 4.0f;

    try
    {
        dt = 0.0f;
        while (true)
        {
            dt += 0.002f;

            if (dt < total_time)
            {
                for (std::size_t i = 0; i < motor_count; ++i)
                {
                    pos_target[i] = pos[i] + (pos_end[i] - pos[i]) * (dt / total_time);
                    kp_target[i]  = kp[i] + (kp_end[i] - kp[i]) * (dt / total_time);
                }

                for (std::size_t i = 0; i < motor_count; ++i)
                {
                    data->motor_cmds[i]->pos = pos_target[i];
                    data->motor_cmds[i]->vel = vel[i];
                    data->motor_cmds[i]->tau = tau[i];
                    data->motor_cmds[i]->kp  = kp_target[i];
                    data->motor_cmds[i]->kd  = kd[i];
                }
            }
            else
            {
                break;
            }

            joint_rviz_state.header.stamp = robot->now();
            for (std::size_t i = 0; i < motor_count; ++i)
            {
                joint_rviz_state.position[i] = data->motor_states[i]->pos;
                joint_rviz_state.velocity[i] = data->motor_states[i]->vel;
                joint_rviz_state.effort[i]   = data->motor_states[i]->tau;
            }
            joint_rviz_pub->publish(joint_rviz_state);

            motor_states.header.stamp = robot->now();
            for (std::size_t i = 0; i < motor_count; ++i)
            {
                motor_states.states[i].pos = data->motor_states[i]->pos;
                motor_states.states[i].vel = data->motor_states[i]->vel;
                motor_states.states[i].tau = data->motor_states[i]->tau;
            }
            joint_state_pub->publish(motor_states);

            rclcpp::spin_some(robot);

            fps_counter.update();

            std::this_thread::sleep_until(next_time);
            next_time += interval;
        }
    }
    catch (std::runtime_error&)
    {
        spdlog::warn("runtime error!");
    }

    auto motor_cmd_sub = robot->create_subscription<robotsky_interface::msg::MotorCmds>("motor_cmds", 10, motor_cmd_callback);

    try
    {
        while (rclcpp::ok())
        {
            joint_rviz_state.header.stamp = robot->now();
            for (std::size_t i = 0; i < motor_count; ++i)
            {
                joint_rviz_state.position[i] = data->motor_states[i]->pos;
                joint_rviz_state.velocity[i] = data->motor_states[i]->vel;
                joint_rviz_state.effort[i]   = data->motor_states[i]->tau;
            }
            joint_rviz_pub->publish(joint_rviz_state);

            motor_states.header.stamp = joint_rviz_state.header.stamp;
            for (std::size_t i = 0; i < motor_count; ++i)
            {
                motor_states.states[i].pos = data->motor_states[i]->pos;
                motor_states.states[i].vel = data->motor_states[i]->vel;
                motor_states.states[i].tau = data->motor_states[i]->tau;
            }
            joint_state_pub->publish(motor_states);

            rclcpp::spin_some(robot);

            fps_counter.update();

            std::this_thread::sleep_until(next_time);
            next_time += interval;
        }
    }
    catch (std::runtime_error&)
    {
        spdlog::warn("runtime error!");
    }

    motor_cmd_sub.reset();

    try
    {
        dt = 0.0f;
        while (true)
        {
            dt += 0.002f;

            if (dt < total_time)
            {
                for (std::size_t i = 0; i < motor_count; ++i)
                {
                    pos_target[i] = pos_end[i] + (pos[i] - pos_end[i]) * (dt / total_time);
                    kp_target[i]  = kp_end[i] + (kp[i] - kp_end[i]) * (dt / total_time);
                }

                for (std::size_t i = 0; i < motor_count; ++i)
                {
                    data->motor_cmds[i]->pos = pos_target[i];
                    data->motor_cmds[i]->vel = vel[i];
                    data->motor_cmds[i]->tau = tau[i];
                    data->motor_cmds[i]->kp  = kp_target[i];
                    data->motor_cmds[i]->kd  = kd[i];
                }
            }
            else
            {
                break;
            }

            fps_counter.update();

            std::this_thread::sleep_until(next_time);
            next_time += interval;
        }
    }
    catch (std::runtime_error&)
    {
        spdlog::warn("runtime error!");
    }

    for (auto can_bus : data->can_buses)
    {
        can_bus->stop();
    }

    for (auto can_bus : data->can_buses)
    {
        can_bus->disable();
    }

    std::this_thread::sleep_for(200ms);

    for (auto can : data->can_interfaces)
    {
        can->finalize();
    }

    rclcpp::shutdown();

    return 0;
}
