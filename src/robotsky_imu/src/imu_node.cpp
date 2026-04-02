#include "imu/ahrs_driver.h"

#include <rclcpp/rclcpp.hpp>

#include <spdlog/spdlog.h>
#include <memory>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        std::shared_ptr<FDILink::AhrsBringup> imu = std::make_shared<FDILink::AhrsBringup>();
        imu->initialize();

        while (rclcpp::ok())
        {
            imu->processLoop();
            rclcpp::spin_some(imu);
        }
    }
    catch (const std::exception& e)
    {
        spdlog::error("imu_node exited with error: {}", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
