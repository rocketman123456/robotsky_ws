#include "can/can_data.h"
#include "motor/motor_data.h"
#include "robot/robot.h"
#include "utils/fps_counter.h"
#include "utils/utils.h"

#include <rclcpp/rclcpp.hpp>

#include <spdlog/spdlog.h>

#include <chrono>
#include <memory>
#include <thread>

// using namespace std::chrono;
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

    motor_infos.push_back({MotorType::DM, MotorMode::POSITION, 0, 0x01, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::POSITION, 0, 0x02, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::POSITION, 0, 0x05, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::POSITION, 0, 0x08, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});

    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 2, 0x02, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 2, 0x03, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 2, 0x06, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 2, 0x07, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});

    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 3, 0x0a, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 3, 0x0b, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 3, 0x0e, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 3, 0x0f, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});

    motor_infos.push_back({MotorType::DM, MotorMode::VELOCITY, 1, 0x09, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::VELOCITY, 1, 0x0c, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::VELOCITY, 1, 0x0d, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::VELOCITY, 1, 0x10, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});

    return motor_infos;
}

std::vector<CanBusInitInfo> prepare_can_bus()
{
    std::vector<CanBusInitInfo> can_bus_infos;

    can_bus_infos.push_back({
        CanType::DM,
        0,
        {0, 1},
        {0, 1, 2, 3, 12, 13, 14, 15},
    });
    can_bus_infos.push_back({
        CanType::RS,
        1,
        {2, 3},
        {4, 5, 6, 7, 8, 9, 10, 11},
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

    robot->initCAN(can_infos);
    robot->initMotors(motor_infos);
    robot->initCANBus(can_bus_infos);
    robot->initRosInterfaces(motor_infos.size());
    robot->start();

    constexpr double kFrequencyHz = 500.0;
    const auto       interval     = Duration(1.0 / kFrequencyHz);
    auto             next_time    = Clock::now() + interval;

    auto thread_id     = std::this_thread::get_id();
    auto native_handle = *reinterpret_cast<std::thread::native_handle_type*>(&thread_id);
    set_thread(0, native_handle);

    FPSCounter fps_counter(true);
    fps_counter.start();

    try
    {
        while (rclcpp::ok())
        {
            robot->tickStateMachine();

            rclcpp::spin_some(robot);

            fps_counter.update();

            std::this_thread::sleep_until(next_time);
            next_time += interval;
        }
    }
    catch (const std::runtime_error& e)
    {
        spdlog::error("Runtime error in main loop: {}", e.what());
    }

    robot->stop();

    rclcpp::shutdown();
}
