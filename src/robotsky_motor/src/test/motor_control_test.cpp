#include "can/can_bus_factory.h"
#include "can/can_data.h"
#include "motor/motor_data.h"
#include "motor/motor_factory.h"
#include "robot/robot.h"
#include "utils/fps_counter.h"
#include "utils/utils.h"

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <tf2_ros/transform_broadcaster.h>

#include <spdlog/spdlog.h>

#include <chrono>
#include <memory>
#include <thread>

using Clock     = std::chrono::high_resolution_clock;
using TimePoint = Clock::time_point;
using Duration  = std::chrono::duration<double>;

sensor_msgs::msg::JointState joint_rviz_state;

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

    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 2, 0x02, -1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 2, 0x03, -1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 2, 0x06, +1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 2, 0x07, +1.0, 0.0, 0.0, 1.0, 1.0, 1.0});

    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 3, 0x0a, -1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 3, 0x0b, -1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 3, 0x0e, +1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 3, 0x0f, +1.0, 0.0, 0.0, 1.0, 1.0, 1.0});

    return motor_infos;
}

std::vector<CanBusInitInfo> prepare_can_bus()
{
    std::vector<CanBusInitInfo> can_bus_infos;

    can_bus_infos.push_back({CanType::DM, 1, {0, 1}, {0, 1, 2, 3, 4, 5, 6, 7}});
    can_bus_infos.push_back({CanType::RS, 2, {2, 3}, {8, 9, 10, 11, 12, 13, 14, 15}});

    return can_bus_infos;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto robot = std::make_shared<Robot>();

    auto joint_rviz_pub  = robot->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

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

    std::vector<CanInitInfo>    can_infos     = prepare_can();
    std::vector<MotorInitInfo>  motor_infos   = prepare_motor();
    std::vector<CanBusInitInfo> can_bus_infos = prepare_can_bus();

    std::shared_ptr<RobotData> data = std::make_shared<RobotData>();

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

    // clang-format off
    uint16_t dm_motor_count = 8;
    float pos_dm[] = {
         0.1, 0.0, // RF
        -0.1, 0.0, // LF
         0.1, 0.0, // RB
        -0.1, 0.0, // LB
    };
    float vel_dm[] = {
        0.0, 0.0, // RF
        0.0, 0.0, // LF
        0.0, 0.0, // RB
        0.0, 0.0, // LB
    };
    float kp_dm[] = {
        30.0, 0.0, // RF
        30.0, 0.0, // LF
        30.0, 0.0, // RB
        30.0, 0.0, // LB
    };
    float kd_dm[] = {
        1.0, 2.0, // RF
        1.0, 2.0, // LF
        1.0, 2.0, // RB
        1.0, 2.0, // LB
    };

    uint16_t rs_motor_count = 8;
    float pos_rs[] = {
        -0.8,  1.6, // RF
        -0.8,  1.6, // LF
         0.8, -1.6, // RB
         0.8, -1.6, // LB
    };
    float kp_rs[] = {
        30.0, 60.0, // RF
        30.0, 60.0, // LF
        30.0, 60.0, // RB
        30.0, 60.0, // LB
    };
    float kd_rs[] = {
        1.0, 2.0, // RF
        1.0, 2.0, // LF
        1.0, 2.0, // RB
        1.0, 2.0, // LB
    };
    // clang-format on

    // for (int i = 0; i < 10; ++i)
    // {
    //     data->can_buses[0]->enable(); // DM
    //     data->can_buses[1]->enable(); // RS
    // }
    data->can_buses[0]->enable(); // DM
    // data->can_buses[1]->enable(); // RS
    // for (auto can_bus : data->can_buses)
    // {
    //     can_bus->enable();
    // }

    double frequency_hz = 200; // 500
    auto   interval     = Duration(1.0 / frequency_hz);
    auto   next_time    = Clock::now() + interval;

    auto thread_id     = std::this_thread::get_id();
    auto native_handle = *reinterpret_cast<std::thread::native_handle_type*>(&thread_id);
    set_thread(0, native_handle);

    FPSCounter fps_counter(true);
    fps_counter.start();

    try
    {
        while (rclcpp::ok())
        {
            // TODO : task
            // robot send cmd to motor
            // robot update motor state
            // send robot joint state msg

            data->can_buses[0]->step(); // DM
            // data->can_buses[1]->step(); // RS
            // for (auto can_bus : data->can_buses)
            // {
            //     can_bus->step();
            // }

            joint_rviz_state.header.stamp = robot->now();
            for (int i = 0; i < motor_count; ++i)
            {
                // joint_rviz_state.position[i] = data->motor_states[data->motors[i]->id - 1]->pos;
                // joint_rviz_state.velocity[i] = data->motor_states[data->motors[i]->id - 1]->vel;
                // joint_rviz_state.effort[i]   = data->motor_states[data->motors[i]->id - 1]->tau;

                joint_rviz_state.position[i] = data->motor_states[i]->pos;
                joint_rviz_state.velocity[i] = data->motor_states[i]->vel;
                joint_rviz_state.effort[i]   = data->motor_states[i]->tau;
            }
            // uint16_t id = 5;
            // spdlog::info("motor {} - {} : pos {}", id, joint_rviz_state.name[id - 1], data->motor_states[id - 1]->pos);
            // joint_rviz_state.position[0] = 1.0;
            joint_rviz_pub->publish(joint_rviz_state);

            rclcpp::spin_some(robot);

            fps_counter.update();

            // 等待直到下一个时间点
            std::this_thread::sleep_until(next_time);
            next_time += interval;
        }
    }
    catch (std::runtime_error& e)
    {
        spdlog::warn("runtime error!");
    }

    data->can_buses[0]->disable(); // DM
    // data->can_buses[1]->disable(); // RS
    // for (auto can_bus : data->can_buses)
    // {
    //     can_bus->disable();
    // }

    // for (auto can : data->can_interfaces)
    // {
    //     can->finalize();
    // }

    rclcpp::shutdown();

    return 0;
}