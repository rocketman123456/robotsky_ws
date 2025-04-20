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

    can_infos.push_back({"can0"});
    can_infos.push_back({"can1"});

    driver.initialize(can_infos);

    can_frame can_tx;
    can_frame can_rx;

    // uint16_t can_ids[]    = {0x01, 0x04};
    // uint16_t can_indexs[] = {0, 0};
    // uint16_t can_ids[]    = {0x05, 0x08};
    // uint16_t can_indexs[] = {0, 0};
    // uint16_t can_ids[]    = {0x09, 0x0c};
    // uint16_t can_indexs[] = {1, 1};
    uint16_t can_ids[]    = {0x0d, 0x10};
    uint16_t can_indexs[] = {1, 1};

    uint16_t motor_count = 2;

    dm_motor_fb_t data;

    for (int i = 0; i < motor_count; ++i)
    {
        dm_save_pos_zero(can_tx, can_ids[i], DM_MIT_MODE);
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