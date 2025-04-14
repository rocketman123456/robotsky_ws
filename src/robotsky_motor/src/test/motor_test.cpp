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

    float delta   = 360.0 / 7.75 / 180.0 * M_PI;
    float offset1 = 30.0 / 180.0 * M_PI;
    float offset2 = 61.0 / 180.0 * M_PI;
    float offset3 = 159.0 / 180.0 * M_PI;

    spdlog::debug("init can port");
    can_infos.emplace_back("can0"); // RF, LF
    can_infos.emplace_back("can2"); // RB, LB
    driver->initialize(can_infos);

    spdlog::debug("init leg motors");
    motor_infos.emplace_back(0, 1, -1.0, -offset1, delta); // RF
    motor_infos.emplace_back(0, 2, 1.0, offset2, delta);
    motor_infos.emplace_back(0, 3, -1.0, -offset3, delta, 2.0, 2.0, 2.0);

    motor_infos.emplace_back(0, 5, -1.0, offset1, delta); // LF
    motor_infos.emplace_back(0, 6, -1.0, offset2, delta);
    motor_infos.emplace_back(0, 7, 1.0, -offset3, delta, 2.0, 2.0, 2.0);

    motor_infos.emplace_back(1, 9, 1.0, -offset1, delta); // RB
    motor_infos.emplace_back(1, 10, 1.0, -offset2, delta);
    motor_infos.emplace_back(1, 11, -1.0, offset3, delta, 2.0, 2.0, 2.0);

    motor_infos.emplace_back(1, 13, 1.0, offset1, delta); // LB
    motor_infos.emplace_back(1, 14, -1.0, -offset2, delta);
    motor_infos.emplace_back(1, 15, 1.0, offset3, delta, 2.0, 2.0, 2.0);
    
    driver->initialize(motor_infos);

    return driver;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto driver = init_wheel_driver();

    spdlog::debug("enable motors");
    driver->enableMotors();
    driver->update();

    rclcpp::Rate loop_rate(100);
    try
    {
        while (rclcpp::ok())
        {
            for(int i = 0; i < 12; ++i)
            {
                driver->getMotor(i).setMixedControlInDeg(0.0, 0.0, 0.0, 0.0, 2.0);
            }
            driver->update();

            spdlog::info("motor 2 pos : {}", driver->getMotor(2).data.pos);
            spdlog::info("motor 5 pos : {}", driver->getMotor(5).data.pos);
            spdlog::info("motor 8 pos : {}", driver->getMotor(8).data.pos);
            spdlog::info("motor 11 pos : {}", driver->getMotor(11).data.pos);

            loop_rate.sleep();
        }
    }
    catch(std::runtime_error& e)
    {
        spdlog::warn("runtime error!");
    }
    

    spdlog::debug("disable motors");
    driver->disableMotors();
    driver->update();

    rclcpp::shutdown();
    return 0;
}