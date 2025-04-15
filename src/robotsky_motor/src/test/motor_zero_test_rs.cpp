#include "can/old/can_driver.h"
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

    can_infos.emplace_back("can2");
    can_infos.emplace_back("can3");

    driver.initialize(can_infos);

    can_frame can_tx;
    can_frame can_rx;

    uint16_t can_ids[]    = {0x02, 0x03};
    uint16_t can_indexs[] = {0, 0};
    // uint16_t can_ids[]    = {0x06, 0x07};
    // uint16_t can_indexs[] = {0, 0};
    // uint16_t can_ids[]    = {0x0a, 0x0b};
    // uint16_t can_indexs[] = {1, 1};
    // uint16_t can_ids[]    = {0x0e, 0x0f};
    // uint16_t can_indexs[] = {1, 1};

    uint16_t motor_count = 2;

    rs_motor_fb_t      data;
    rs_data_read_write data_motor;

    for (int i = 0; i < motor_count; ++i)
    {
        rs_save_pos_zero(can_tx, can_ids[i]);
        {
            driver.send(can_indexs[i], can_tx);
            usleep(50);
            driver.receive(can_indexs[i], can_rx);
        }
        rs_decode(can_rx, data, data_motor);
    }

    rclcpp::shutdown();
    return 0;
}