#include "hardware/leg_driver.h"

#include <rclcpp/rclcpp.hpp>

#include <spdlog/spdlog.h>

#include <cmath>
#include <cstdio>
#include <memory>
#include <unistd.h> // usleep
#include <vector>

bool is_near(float a, float b, float tol = 1e-4) { return abs(abs(a) - abs(b)) < tol; }

std::shared_ptr<LegDriver> init_wheel_driver()
{
    std::shared_ptr<LegDriver> driver = std::make_shared<LegDriver>();

    std::vector<can_init_info_t>   can_infos;
    std::vector<motor_init_info_t> motor_infos;

    spdlog::debug("init can port");
    can_infos.emplace_back("can0"); // RF, LF
    can_infos.emplace_back("can2"); // RB, LB
    driver->initialize(can_infos);

    float delta = 360.0 / 7.75 / 180.0 * M_PI;

    spdlog::debug("init leg motors");
    motor_infos.emplace_back(0, 1, 1.0, delta); // RF
    motor_infos.emplace_back(0, 2, 1.0, delta);
    motor_infos.emplace_back(0, 3, 1.0, delta);

    motor_infos.emplace_back(0, 5, 1.0, delta); // LF
    motor_infos.emplace_back(0, 6, 1.0, delta);
    motor_infos.emplace_back(0, 7, 1.0, delta);

    motor_infos.emplace_back(1, 9, 1.0, delta); // RB
    motor_infos.emplace_back(1, 10, 1.0, delta);
    motor_infos.emplace_back(1, 11, 1.0, delta);

    motor_infos.emplace_back(1, 13, 1.0, delta); // LB
    motor_infos.emplace_back(1, 14, 1.0, delta);
    motor_infos.emplace_back(1, 15, 1.0, delta);
    
    driver->initialize(motor_infos);

    return driver;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto driver = init_wheel_driver();

    spdlog::debug("set zero motors");
    driver->setMotorsZero();
    driver->update();

    spdlog::debug("enable motors");
    driver->enableMotors();
    driver->update();

    spdlog::debug("disable motors");
    driver->disableMotors();
    driver->update();

    rclcpp::shutdown();
    return 0;
}