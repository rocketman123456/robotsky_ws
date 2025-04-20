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

    can_infos.push_back({"can2"});
    can_infos.push_back({"can3"});

    driver.initialize(can_infos);

    can_frame can_tx;
    can_frame can_rx;

    uint16_t can_ids[]    = {0x02, 0x03, 0x06, 0x07, 0x0a, 0x0b, 0x0e, 0x0f};
    uint16_t can_indexs[] = {0, 0, 0, 0, 1, 1, 1, 1};

    float pos[] = {
        -0.6,
        1.2, // RF
        0.6,
        -1.2, // LF
        0.6,
        -1.2, // RB
        -0.6,
        1.2, // LB
    };
    float kp[] = {
        0.0,
        0.0, // RF
        0.0,
        0.0, // LF
        0.0,
        0.0, // RB
        0.0,
        0.0, // LB
    };
    float kd[] = {
        1.0,
        1.0, // RF
        1.0,
        1.0, // LF
        1.0,
        1.0, // RB
        1.0,
        1.0, // LB
    };

    uint16_t motor_count = 8;

    rs_motor_fb_t      data;
    rs_data_read_write data_motor;

    for (int i = 0; i < motor_count; ++i)
    {
        rs_enable_motor_mode(can_tx, can_ids[i]);
        {
            driver.send(can_indexs[i], can_tx);
            usleep(50);
            driver.receive(can_indexs[i], can_rx);
            usleep(50);
        }
        rs_decode(can_rx, data, data_motor);
    }

    for (int i = 0; i < motor_count; ++i)
    {
        rs_set_motor_parameter(can_tx, can_ids[i], 0X7005, RS_Move_Control_mode, RS_Set_mode);
        {
            driver.send(can_indexs[i], can_tx);
            usleep(50);
            driver.receive(can_indexs[i], can_rx);
            // usleep(50);
        }
        rs_decode(can_rx, data, data_motor);
    }

    // for (int i = 0; i < motor_count; ++i)
    // {
    //     rs_get_motor_parameter(can_tx, can_ids[i], 0X7005);
    //     {
    //         driver.send(can_indexs[i], can_tx);
    //         usleep(50);
    //         driver.receive(can_indexs[i], can_rx);
    //         usleep(50);
    //     }
    //     rs_decode(can_rx, data, data_motor);
    // }

    spdlog::info("start");

    rclcpp::Rate loop_rate(200);
    try
    {
        while (rclcpp::ok())
        {
            for (int i = 0; i < motor_count; ++i)
            {
                rs_mit_ctrl(can_tx, can_ids[i], pos[i], 0.0f, kp[i], kd[i], 0.0f);
                {
                    driver.send(can_indexs[i], can_tx);
                    usleep(50);
                    driver.receive(can_indexs[i], can_rx);
                    // usleep(50);
                }
                rs_decode(can_rx, data, data_motor);

                if (data.id == can_ids[0])
                    spdlog::info("motor {} pos : {}", data.id, data.pos);
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
        rs_disable_motor_mode(can_tx, can_ids[i], 0);
        {
            driver.send(can_indexs[i], can_tx);
            usleep(50);
            driver.receive(can_indexs[i], can_rx);
            usleep(50);
        }
        rs_decode(can_rx, data, data_motor);
    }

    rclcpp::shutdown();
    return 0;
}
