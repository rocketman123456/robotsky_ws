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
    uint16_t  can_id    = 0x03;
    uint16_t  can_index = 0;

    rs_enable_motor_mode(can_tx, can_id);
    {
        driver.send(can_index, can_tx);
        usleep(50);
        driver.receive(can_index, can_rx);
    }

    rs_set_motor_parameter(can_tx, can_id, 0X7005, RS_Move_Control_mode, RS_Set_mode);
    {
        driver.send(can_index, can_tx);
        usleep(50);
        driver.receive(can_index, can_rx);
    }

    rs_get_motor_parameter(can_tx, can_id, 0X7005);
    {
        driver.send(can_index, can_tx);
        usleep(50);
        driver.receive(can_index, can_rx);
    }

    rs_mit_ctrl(can_tx, can_id, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f);
    {
        driver.send(can_index, can_tx);
        usleep(50);
        driver.receive(can_index, can_rx);
    }
    // dm_decode(can_rx, data);

    spdlog::info("start");

    uint64_t count = 0;

    rclcpp::Rate loop_rate(200);
    try
    {
        while (rclcpp::ok())
        {
            rs_set_motor_parameter(can_tx, can_id, 0X7005, RS_Move_Control_mode, RS_Set_mode);
            {
                driver.send(can_index, can_tx);
                usleep(50);
                driver.receive(can_index, can_rx);
            }

            rs_get_motor_parameter(can_tx, can_id, 0X7005);
            {
                driver.send(can_index, can_tx);
                usleep(50);
                driver.receive(can_index, can_rx);
            }

            rs_mit_ctrl(can_tx, can_id, 0.0f, 0.0f, 0.0f, 1.5f, 0.0f);
            {
                driver.send(can_index, can_tx);
                usleep(50);
                driver.receive(can_index, can_rx);
            }

            // count++;
            // if (count % 10 == 0)
            // {
            //     spdlog::info("tick");
            // }
            // spdlog::info("tick");

            loop_rate.sleep();
        }
    }
    catch (std::runtime_error& e)
    {
        spdlog::warn("runtime error!");
    }

    rs_disable_motor_mode(can_tx, can_id, 0);
    {
        driver.send(can_index, can_tx);
        usleep(50);
        driver.receive(can_index, can_rx);
    }
    // dm_decode(can_rx, data);

    rclcpp::shutdown();
    return 0;
}