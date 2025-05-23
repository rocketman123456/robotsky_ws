#include "can/can_bus_factory.h"
#include "can/can_data.h"
#include "motor/motor_data.h"
#include "motor/motor_factory.h"
#include "robot/robot.h"
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
using Clock     = std::chrono::high_resolution_clock;
using TimePoint = Clock::time_point;
using Duration  = std::chrono::duration<double>;

sensor_msgs::msg::JointState         joint_rviz_state;
robotsky_interface::msg::MotorCmds   motor_cmds;
robotsky_interface::msg::MotorStates motor_states;

std::shared_ptr<RobotData> data = std::make_shared<RobotData>();

const uint16_t motor_count = 16;

std::vector<CanInitInfo> prepare_can()
{
    std::vector<CanInitInfo> can_infos;

    can_infos.push_back({"can0"});
    can_infos.push_back({"can1"});
    can_infos.push_back({"can2"});
    can_infos.push_back({"can3"});

    return can_infos;
}

std::vector<MotorInitInfo> prepare_motor()
{
    std::vector<MotorInitInfo> motor_infos;

    motor_infos.push_back({MotorType::DM, MotorMode::POSITION, 0, 0x01, +1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::VELOCITY, 0, 0x04, -1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::POSITION, 0, 0x05, +1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::VELOCITY, 0, 0x08, +1.0, 0.0, 0.0, 1.0, 1.0, 1.0});

    motor_infos.push_back({MotorType::DM, MotorMode::POSITION, 1, 0x09, -1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::VELOCITY, 1, 0x0c, -1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::POSITION, 1, 0x0d, -1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::VELOCITY, 1, 0x10, +1.0, 0.0, 0.0, 1.0, 1.0, 1.0});

    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 2, 0x02, +1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 2, 0x03, +1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 2, 0x06, -1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 2, 0x07, -1.0, 0.0, 0.0, 1.0, 1.0, 1.0});

    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 3, 0x0a, +1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 3, 0x0b, +1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 3, 0x0e, -1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 3, 0x0f, -1.0, 0.0, 0.0, 1.0, 1.0, 1.0});

    return motor_infos;
}

std::vector<CanBusInitInfo> prepare_can_bus()
{
    std::vector<CanBusInitInfo> can_bus_infos;

    can_bus_infos.push_back({
        CanType::DM, 1, {0, 1},
          {0, 1, 2, 3, 4, 5, 6, 7}
    });
    can_bus_infos.push_back({
        CanType::RS, 2, {2, 3},
          {8, 9, 10, 11, 12, 13, 14, 15}
    });

    return can_bus_infos;
}

void prepare_hardware()
{
    std::vector<CanInitInfo>    can_infos     = prepare_can();
    std::vector<MotorInitInfo>  motor_infos   = prepare_motor();
    std::vector<CanBusInitInfo> can_bus_infos = prepare_can_bus();

    for (int i = 0; i < motor_infos.size(); ++i)
    {
        data->motor_states.push_back(std::make_shared<MotorState>());
        data->motor_cmds.push_back(std::make_shared<MotorCmd>());
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
    for (int i = 0; i < motor_count; ++i)
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
    joint_rviz_state.name.resize(motor_count);

    joint_rviz_state.name = {
        "RF_Roll_Joint",
        "RF_Hip_Joint",
        "RF_Knee_Joint",
        "RF_Wheel_Joint",

        "LF_Roll_Joint",
        "LF_Hip_Joint",
        "LF_Knee_Joint",
        "LF_Wheel_Joint",

        "RB_Roll_Joint",
        "RB_Hip_Joint",
        "RB_Knee_Joint",
        "RB_Wheel_Joint",

        "LB_Roll_Joint",
        "LB_Hip_Joint",
        "LB_Knee_Joint",
        "LB_Wheel_Joint",
    };

    motor_cmds.cmds.resize(motor_count);
    motor_states.states.resize(motor_count);

    prepare_hardware();

    // clang-format off
    float pos_target[] = {
        0.0, 0.0, 0.0, 0.0, // RF
        0.0, 0.0, 0.0, 0.0, // LF
        0.0, 0.0, 0.0, 0.0, // RB
        0.0, 0.0, 0.0, 0.0, // LB
    };
    float pos[] = {
        0.0, 0.0, 0.0, 0.0, // RF
        0.0, 0.0, 0.0, 0.0, // LF
        0.0, 0.0, 0.0, 0.0, // RB
        0.0, 0.0, 0.0, 0.0, // LB
    };
    float pos_end[] = {
        0.0, -0.8,  1.6, 0.0, // RF
        0.0, -0.8,  1.6, 0.0, // LF
        0.0,  0.8, -1.6, 0.0, // RB
        0.0,  0.8, -1.6, 0.0, // LB
    };
    float vel[] = {
        0.0, 0.0, 0.0, 0.0, // RF
        0.0, 0.0, 0.0, 0.0, // LF
        0.0, 0.0, 0.0, 0.0, // RB
        0.0, 0.0, 0.0, 0.0, // LB
    };
    float tau[] = {
        0.0, 0.0, 0.0, 0.0, // RF
        0.0, 0.0, 0.0, 0.0, // LF
        0.0, 0.0, 0.0, 0.0, // RB
        0.0, 0.0, 0.0, 0.0, // LB
    };
    float kp_target[] = {
        0.0, 0.0, 0.0, 0.0, // RF
        0.0, 0.0, 0.0, 0.0, // LF
        0.0, 0.0, 0.0, 0.0, // RB
        0.0, 0.0, 0.0, 0.0, // LB
    };
    float kp[] = {
        0.0, 0.0, 0.0, 0.0, // RF
        0.0, 0.0, 0.0, 0.0, // LF
        0.0, 0.0, 0.0, 0.0, // RB
        0.0, 0.0, 0.0, 0.0, // LB
    };
    float kp_end[] = {
        30.0, 30.0, 60.0, 0.0, // RF
        30.0, 30.0, 60.0, 0.0, // LF
        30.0, 30.0, 60.0, 0.0, // RB
        30.0, 30.0, 60.0, 0.0, // LB
    };
    float kd[] = {
        1.0, 1.0, 2.0, 2.0, // RF
        1.0, 1.0, 2.0, 2.0, // LF
        1.0, 1.0, 2.0, 2.0, // RB
        1.0, 1.0, 2.0, 2.0, // LB
    };
    // clang-format on

    for (int i = 0; i < motor_count; ++i)
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
        can_bus->start();
    }

    double frequency_hz = 500;
    auto   interval     = Duration(1.0 / frequency_hz);
    auto   next_time    = Clock::now() + interval;

    auto thread_id     = std::this_thread::get_id();
    auto native_handle = *reinterpret_cast<std::thread::native_handle_type*>(&thread_id);
    set_thread(0, native_handle);

    FPSCounter fps_counter;
    fps_counter.start();

    float dt         = 0.0;
    float total_time = 4.0;

    try
    {
        dt = 0;
        while (true)
        {
            dt += 0.002;

            if (dt < total_time)
            {
                for (int i = 0; i < motor_count; ++i)
                {
                    pos_target[i] = pos[i] + (pos_end[i] - pos[i]) * (dt / total_time);
                    kp_target[i]  = kp[i] + (kp_end[i] - kp[i]) * (dt / total_time);
                }

                for (int i = 0; i < motor_count; ++i)
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
            for (int i = 0; i < motor_count; ++i)
            {
                joint_rviz_state.position[i] = data->motor_states[i]->pos;
                joint_rviz_state.velocity[i] = data->motor_states[i]->vel;
                joint_rviz_state.effort[i]   = data->motor_states[i]->tau;
            }
            joint_rviz_pub->publish(joint_rviz_state);

            motor_states.header.stamp = robot->now();
            for (int i = 0; i < motor_count; ++i)
            {
                motor_states.states[i].pos = data->motor_states[i]->pos;
                motor_states.states[i].vel = data->motor_states[i]->vel;
                motor_states.states[i].tau = data->motor_states[i]->tau;
            }
            joint_state_pub->publish(motor_states);

            rclcpp::spin_some(robot);

            std::this_thread::sleep_until(next_time);
            next_time += interval;
        }
    }
    catch (std::runtime_error& e)
    {
        spdlog::warn("runtime error!");
    }

    auto motor_cmd_sub = robot->create_subscription<robotsky_interface::msg::MotorCmds>("motor_cmds", 10, motor_cmd_callback);

    try
    {
        while (rclcpp::ok())
        {
            joint_rviz_state.header.stamp = robot->now();
            for (int i = 0; i < motor_count; ++i)
            {
                joint_rviz_state.position[i] = data->motor_states[i]->pos;
                joint_rviz_state.velocity[i] = data->motor_states[i]->vel;
                joint_rviz_state.effort[i]   = data->motor_states[i]->tau;
            }
            joint_rviz_pub->publish(joint_rviz_state);

            motor_states.header.stamp = joint_rviz_state.header.stamp;
            for (int i = 0; i < motor_count; ++i)
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
    catch (std::runtime_error& e)
    {
        spdlog::warn("runtime error!");
    }

    motor_cmd_sub.reset();

    try
    {
        dt = 0;
        while (true)
        {
            dt += 0.002;

            if (dt < total_time)
            {
                for (int i = 0; i < motor_count; ++i)
                {
                    pos_target[i] = pos_end[i] + (pos[i] - pos_end[i]) * (dt / total_time);
                    kp_target[i]  = kp_end[i] + (kp[i] - kp_end[i]) * (dt / total_time);
                }

                for (int i = 0; i < motor_count; ++i)
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

            std::this_thread::sleep_until(next_time);
            next_time += interval;
        }
    }
    catch (std::runtime_error& e)
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