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

    can_infos.emplace_back("can2");
    can_infos.emplace_back("can3");

    driver.initialize(can_infos);

    can_frame can_tx;
    can_frame can_rx;
    uint16_t  can_id    = 0x0a;
    uint16_t  can_index = 1;

    rs_motor_fb_t      data;
    rs_data_read_write data_motor;

    rs_enable_motor_mode(can_tx, can_id);
    {
        driver.send(can_index, can_tx);
        usleep(50);
        driver.receive(can_index, can_rx);
    }
    rs_decode(can_rx, data, data_motor);

    rs_set_motor_parameter(can_tx, can_id, 0X7005, RS_Move_Control_mode, RS_Set_mode);
    {
        driver.send(can_index, can_tx);
        usleep(50);
        driver.receive(can_index, can_rx);
    }
    rs_decode(can_rx, data, data_motor);

    rs_get_motor_parameter(can_tx, can_id, 0X7005);
    {
        driver.send(can_index, can_tx);
        usleep(50);
        driver.receive(can_index, can_rx);
    }
    rs_decode(can_rx, data, data_motor);

    rs_mit_ctrl(can_tx, can_id, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f);
    {
        driver.send(can_index, can_tx);
        usleep(50);
        driver.receive(can_index, can_rx);
    }
    rs_decode(can_rx, data, data_motor);

    spdlog::info("start");

    rclcpp::Rate loop_rate(200);
    try
    {
        while (rclcpp::ok())
        {
            // rs_set_motor_parameter(can_tx, can_id, 0X7005, RS_Move_Control_mode, RS_Set_mode);
            // {
            //     driver.send(can_index, can_tx);
            //     usleep(50);
            //     driver.receive(can_index, can_rx);
            // }

            // rs_get_motor_parameter(can_tx, can_id, 0X7005);
            // {
            //     driver.send(can_index, can_tx);
            //     usleep(50);
            //     driver.receive(can_index, can_rx);
            // }

            rs_mit_ctrl(can_tx, can_id, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f);
            {
                driver.send(can_index, can_tx);
                usleep(50);
                driver.receive(can_index, can_rx);
            }
            rs_decode(can_rx, data, data_motor);

            spdlog::info("motor {} pos : {}", data.id, data.pos);

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