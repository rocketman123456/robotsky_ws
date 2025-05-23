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

    uint16_t can_ids[]    = {0x01, 0x04, 0x05, 0x08, 0x09, 0x0c, 0x0d, 0x10};
    uint16_t can_indexs[] = {0, 0, 0, 0, 1, 1, 1, 1};

    // clang-format off
    float pos[] = {
        0.0, 0.0, // RF
        0.0, 0.0, // LF
        0.0, 0.0, // RB
        0.0, 0.0, // LB
    };
    float vel[] = {
        0.0, -0.0, // RF
        0.0, 0.0, // LF
        0.0, -0.0, // RB
        0.0, 0.0, // LB
    };
    float kp[] = {
        0.0, 0.0, // RF
        0.0, 0.0, // LF
        0.0, 0.0, // RB
        0.0, 0.0, // LB
    };
    float kd[] = {
        1.0, 2.0, // RF
        1.0, 2.0, // LF
        1.0, 2.0, // RB
        1.0, 2.0, // LB
    };
    // clang-format on

    float pos_fb[16] = {0};
    float vel_fb[16] = {0};
    float tau_fb[16] = {0};

    uint16_t motor_count = 8;

    dm_motor_fb_t data;

    for (int i = 0; i < motor_count; ++i)
    {
        dm_enable_motor_mode(can_tx, can_ids[i], DM_MIT_MODE);
        {
            driver.send(can_indexs[i], can_tx);
            usleep(50);
            driver.receive(can_indexs[i], can_rx);
            usleep(50);
        }
        dm_decode(can_rx, data);
    }

    spdlog::info("start");

    try
    {
        rclcpp::Rate loop_rate(100);
        while (rclcpp::ok())
        {
            for (int i = 0; i < motor_count; ++i)
            {
                dm_mit_ctrl(can_tx, can_ids[i], pos[i], vel[i], kp[i], kd[i], 0.0f);
                {
                    driver.send(can_indexs[i], can_tx);
                    usleep(50);
                    driver.receive(can_indexs[i], can_rx);
                    usleep(50);
                }
                dm_decode(can_rx, data);

                // if(data.id == can_ids[6])
                //     spdlog::info("motor {} pos : {}", data.id, data.pos);

                // spdlog::info("motor {} - pos : {}, vel : {}, tau : {}", can_ids[i], pos_fb[can_ids[i] - 1], vel_fb[can_ids[i] - 1], tau_fb[can_ids[i] - 1]);
                spdlog::info("motor {} - {} - pos : {}, vel : {}", data.mst_id - 0x40, data.id, data.pos, data.vel);

                pos_fb[data.id - 1] = data.pos;
                vel_fb[data.id - 1] = data.vel;
                // tau_fb[data.id - 1] = data.tau;
            }

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
            usleep(50);
        }
        dm_decode(can_rx, data);
    }

    rclcpp::shutdown();
    return 0;
}
