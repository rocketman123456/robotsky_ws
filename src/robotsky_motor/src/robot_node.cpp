#include "robot/robot.h"
#include "robot/robotsky_wheel_leg_hardware.h"
#include "utils/fps_counter.h"
#include "utils/utils.h"

#include <rclcpp/rclcpp.hpp>

#include <spdlog/spdlog.h>

#include <chrono>
#include <memory>
#include <thread>

using Clock     = std::chrono::high_resolution_clock;
using Duration  = std::chrono::duration<double>;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto robot = std::make_shared<Robot>();

    const auto can_infos     = robotsky::wheel_leg::prepareCan();
    const auto motor_infos   = robotsky::wheel_leg::prepareMotors();
    const auto can_bus_infos = robotsky::wheel_leg::prepareCanBuses();

    robot->initCAN(can_infos);
    robot->initMotors(motor_infos);
    robot->initCANBus(can_bus_infos);
    robot->initRosInterfaces(robotsky::wheel_leg::kMotorCount);
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

    for (auto& can_bus : robot->data->can_buses)
    {
        can_bus->disable();
    }

    for (auto& can_interface : robot->data->can_interfaces)
    {
        can_interface->finalize();
    }

    rclcpp::shutdown();
    return 0;
}
