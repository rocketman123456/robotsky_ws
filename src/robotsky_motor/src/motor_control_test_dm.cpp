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

    std::vector<CanInitInfo> can_infos;

    can_infos.emplace_back("can0");
    can_infos.emplace_back("can1");

    driver.initialize(can_infos);

    can_frame can_tx;
    can_frame can_rx;

    // uint16_t  can_id    = 0x0c;
    // uint16_t  can_index = 1;

    uint16_t can_ids[]    = {0x01, 0x04, 0x05, 0x08, 0x09, 0x0c, 0x0d, 0x10};
    uint16_t can_indexs[] = {0, 0, 0, 0, 1, 1, 1, 1};

    uint16_t motor_count = 8;

    dm_motor_fb_t data;

    for (int i = 0; i < motor_count; ++i)
    {
        dm_enable_motor_mode(can_tx, can_ids[i], DM_MIT_MODE);
        {
            driver.send(can_indexs[i], can_tx);
            usleep(50);
            driver.receive(can_indexs[i], can_rx);
        }
        dm_decode(can_rx, data);
    }

    rclcpp::Rate loop_rate(100);
    try
    {
        while (rclcpp::ok())
        {
            for (int i = 0; i < motor_count; ++i)
            {
                dm_mit_ctrl(can_tx, can_ids[i], 0.0f, 0.0f, 0.0f, 2.0f, 0.0f);
                {
                    driver.send(can_indexs[i], can_tx);
                    usleep(50);
                    driver.receive(can_indexs[i], can_rx);
                }
                dm_decode(can_rx, data);
            }

            // spdlog::info("motor {} pos : {}", data.id, data.pos);

            loop_rate.sleep();
        }
    }
    catch (std::runtime_error& e)
    {
        spdlog::warn("runtime error!");
    }

    for (int i = 0; i < motor_count; ++i)
    {
        dm_disable_motor_mode(can_tx, can_ids[i], DM_MIT_MODE);
        {
            driver.send(can_indexs[i], can_tx);
            usleep(50);
            driver.receive(can_indexs[i], can_rx);
        }
        dm_decode(can_rx, data);
    }

    rclcpp::shutdown();
    return 0;
}