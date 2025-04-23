#include "can/bus/dm_can_bus_manager.h"
#include "motor/utils/dm_motor_utils.h"
#include "robot/robot_data.h"

#include <spdlog/spdlog.h>

#include <unistd.h> // usleep

DMCANBusManager::DMCANBusManager()
    : CANBusManager()
{
    spdlog::info("DMCANBusManager init");
    type = CanType::DM;
}

// I use special master id for each motor: mst_id = id + 0x40, so i can decode it from can id

void DMCANBusManager::enable()
{
    spdlog::info("DMCANBusManager enable");

    dm_motor_fb_t data_fb;
    can_frame     can_rx;

    for (auto index : motor_indices)
    {
        auto motor = data->motors[index];
        auto can   = data->can_interfaces[motor->can_index];

        motor->enable();

        can->send(motor->can_tx);
        usleep(50);
        can->receive(motor->can_rx);
        usleep(50);

        dm_decode(motor->can_rx, data_fb);

        uint16_t id = data_fb.mst_id - 1 - 0x40;

        spdlog::info("motor {} - pos : {}, vel : {}", id + 1, data_fb.pos, data_fb.vel);

        if (id >= 0 && id < data->motor_states.size())
        {
            // TODO : lock
            data->motor_states[id]->pos = data_fb.pos;
            data->motor_states[id]->vel = data_fb.vel;
        }
        else
        {
            spdlog::warn("motor id {} out of range", id + 1);
        }
    }
}

void DMCANBusManager::disable()
{
    spdlog::info("DMCANBusManager disable");

    dm_motor_fb_t data_fb;
    can_frame     can_rx;

    for (auto index : motor_indices)
    {
        auto motor = data->motors[index];
        auto can   = data->can_interfaces[motor->can_index];

        motor->disable();

        can->send(motor->can_tx);
        usleep(50);
        can->receive(motor->can_rx);
        usleep(50);

        dm_decode(motor->can_rx, data_fb);

        uint16_t id = data_fb.mst_id - 1 - 0x40;

        spdlog::info("motor {} - pos : {}, vel : {}", id + 1, data_fb.pos, data_fb.vel);

        if (id >= 0 && id < data->motor_states.size())
        {
            // TODO : lock
            data->motor_states[id]->pos = data_fb.pos;
            data->motor_states[id]->vel = data_fb.vel;
        }
        else
        {
            spdlog::warn("motor id {} out of range", id + 1);
        }
    }

    spdlog::info("DMCANBusManager disable finish");
}

void DMCANBusManager::step()
{
    dm_motor_fb_t data_fb;
    can_frame     can_rx;

    for (auto index : motor_indices)
    {
        // dm_mit_ctrl(can_tx, can_ids[i], pos[i], vel[i], kp[i], kd[i], 0.0f);
        // {
        //     driver.send(can_indexs[i], can_tx);
        //     usleep(50);
        //     driver.receive(can_indexs[i], can_rx);
        //     usleep(50);
        // }
        // dm_decode(can_rx, data);

        auto motor = data->motors[index];
        auto can   = data->can_interfaces[motor->can_index];

        motor->enable();

        can->send(motor->can_tx);
        usleep(50);
        can->receive(motor->can_rx);
        usleep(50);

        dm_decode(motor->can_rx, data_fb);

        uint16_t id = data_fb.mst_id - 1 - 0x40;

        // spdlog::info("motor {} - pos : {}, vel : {}", id + 1, data_fb.pos, data_fb.vel);

        if (id >= 0 && id < data->motor_states.size())
        {
            // TODO : lock
            data->motor_states[id]->pos = data_fb.pos;
            data->motor_states[id]->vel = data_fb.vel;
        }
        else
        {
            spdlog::warn("motor id {} out of range", id + 1);
        }
    }
}
