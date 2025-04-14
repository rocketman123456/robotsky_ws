#include "imu/ahrs_driver.h"

#include <rclcpp/rclcpp.hpp>

#include <spdlog/spdlog.h>
#include <memory>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<FDILink::AhrsBringup> imu = std::make_shared<FDILink::AhrsBringup>();
    imu->initialize();

    try
    {
        // rclcpp::Rate loop_rate(100);
        while (rclcpp::ok())
        {
            imu->processLoop();

            rclcpp::spin_some(imu);
            // loop_rate.sleep();
        }
    }
    catch(std::runtime_error& e)
    {
        spdlog::warn("runtime error!");
    }

    // rclcpp::spin(imu);

    rclcpp::shutdown();
    return 0;
}
