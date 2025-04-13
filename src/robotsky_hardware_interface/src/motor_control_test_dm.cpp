#include "can/can_driver.h"
#include "motor/utils/dm_motor_utils.h"
#include "motor/utils/rs_motor_utils.h"

#include <rclcpp/rclcpp.hpp>

#include <linux/can.h>
#include <spdlog/spdlog.h>

#include <cmath>
#include <cstdio>
#include <memory>
#include <unistd.h> // usleep
#include <vector>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    CanDriver driver;

    std::vector<can_init_info_t> can_infos;

    can_infos.emplace_back("can0");

    driver.initialize(can_infos);

    can_frame can_tx;
    can_frame can_rx;
    uint16_t  can_id    = 0x01;
    uint16_t  can_index = 0;

    dm_motor_fb_t data;

    dm_enable_motor_mode(can_tx, can_id, DM_MIT_MODE);
    {
        driver.send(can_index, can_tx);
        usleep(50);
        driver.receive(can_index, can_rx);
    }
    dm_decode(can_rx, data);

    rclcpp::Rate loop_rate(100);
    try
    {
        while (rclcpp::ok())
        {
            dm_mit_ctrl(can_tx, can_id, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f);
            {
                driver.send(can_index, can_tx);
                usleep(50);
                driver.receive(can_index, can_rx);
            }
            dm_decode(can_rx, data);

            spdlog::info("motor {} pos : {}", data.id, data.pos);

            loop_rate.sleep();
        }
    }
    catch (std::runtime_error& e)
    {
        spdlog::warn("runtime error!");
    }

    dm_disable_motor_mode(can_tx, can_id, DM_MIT_MODE);
    {
        driver.send(can_index, can_tx);
        usleep(50);
        driver.receive(can_index, can_rx);
    }
    dm_decode(can_rx, data);

    rclcpp::shutdown();
    return 0;
}