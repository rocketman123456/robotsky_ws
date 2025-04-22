#include "can/can_bus_factory.h"
#include "can/can_data.h"
#include "motor/motor_data.h"
#include "motor/motor_factory.h"
#include "robot/robot.h"
#include "utils/fps_counter.h"
#include "utils/utils.h"

#include <rclcpp/rclcpp.hpp>

#include <spdlog/spdlog.h>

#include <chrono>
#include <memory>
#include <thread>

using Clock     = std::chrono::high_resolution_clock;
using TimePoint = Clock::time_point;
using Duration  = std::chrono::duration<double>;

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
    motor_infos.push_back({MotorType::DM, MotorMode::POSITION, 0, 0x02, -1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::POSITION, 0, 0x05, +1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::POSITION, 0, 0x08, +1.0, 0.0, 0.0, 1.0, 1.0, 1.0});

    motor_infos.push_back({MotorType::DM, MotorMode::VELOCITY, 1, 0x09, -1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::VELOCITY, 1, 0x0c, -1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::VELOCITY, 1, 0x0d, -1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
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

    can_bus_infos.push_back({
        CanType::DM,
        1, // cpu core
        {0, 1},
        {0, 1, 2, 3, 4, 5, 6, 7},
    });
    can_bus_infos.push_back({
        CanType::RS,
        2, // cpu core
        {2, 3},
        {8, 9, 10, 11, 12, 13, 14, 15},
    });

    return can_bus_infos;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto robot = std::make_shared<Robot>();

    std::vector<CanInitInfo>    can_infos     = prepare_can();
    std::vector<MotorInitInfo>  motor_infos   = prepare_motor();
    std::vector<CanBusInitInfo> can_bus_infos = prepare_can_bus();

    std::shared_ptr<RobotData> data = std::make_shared<RobotData>();

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

    for (auto can : data->can_interfaces)
    {
        can->finalize();
    }

    rclcpp::shutdown();

    return 0;
}