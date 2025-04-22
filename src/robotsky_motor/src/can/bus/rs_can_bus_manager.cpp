#include "can/bus/rs_can_bus_manager.h"
#include "motor/utils/rs_motor_utils.h"
#include "robot/robot_data.h"

#include <spdlog/spdlog.h>

#include <unistd.h> // usleep

RSCANBusManager::RSCANBusManager()
    : CANBusManager()
{
    spdlog::info("RSCANBusManager init");
    type = CanType::RS;
}

void RSCANBusManager::enable()
{
    spdlog::info("RSCANBusManager enable");

    rs_motor_fb_t      data_fb;
    rs_data_read_write data_motor;

    can_frame can_rx;

    for(auto index : motor_indices)
    {
        auto motor = data->motors[index];
        auto can = data->can_interfaces[motor->can_index];

        motor->enable();

        can->send(motor->can_tx);
        usleep(50);
        can->receive(motor->can_rx);
        usleep(50);

        rs_decode(can_rx, data_fb, data_motor);

        spdlog::info("motor {} - pos : {}, vel : {}", data_fb.id, data_fb.pos, data_fb.vel);

        if (data_fb.id > 0 && data_fb.id <= data->motor_states.size())
        {
            data->motor_states[data_fb.id - 1]->pos = data_fb.pos;
            data->motor_states[data_fb.id - 1]->vel = data_fb.vel;
            data->motor_states[data_fb.id - 1]->tau = data_fb.tau;
        }
        else
        {
            spdlog::warn("motor id {} out of range", data_fb.id);
        }
    }

    spdlog::info("RSCANBusManager enable finish");
}

void RSCANBusManager::disable()
{
    spdlog::info("RSCANBusManager disable");

    rs_motor_fb_t      data_fb;
    rs_data_read_write data_motor;

    can_frame can_rx;

    for(auto index : motor_indices)
    {
        auto motor = data->motors[index];
        auto can = data->can_interfaces[motor->can_index];

        motor->disable();

        can->send(motor->can_tx);
        usleep(50);
        can->receive(motor->can_rx);
        usleep(50);

        rs_decode(can_rx, data_fb, data_motor);

        spdlog::info("motor {} - pos : {}, vel : {}", data_fb.id, data_fb.pos, data_fb.vel);

        if (data_fb.id > 0 && data_fb.id <= data->motor_states.size())
        {
            data->motor_states[data_fb.id - 1]->pos = data_fb.pos;
            data->motor_states[data_fb.id - 1]->vel = data_fb.vel;
            data->motor_states[data_fb.id - 1]->tau = data_fb.tau;
        }
        else
        {
            spdlog::warn("motor id {} out of range", data_fb.id);
        }
    }

    spdlog::info("RSCANBusManager disable finish");
}

void RSCANBusManager::step()
{
    rs_motor_fb_t      data_fb;
    rs_data_read_write data_motor;

    can_frame can_rx;

    for(auto index : motor_indices)
    {
        auto motor = data->motors[index];
        auto can = data->can_interfaces[motor->can_index];

        motor->disable();

        can->send(motor->can_tx);
        usleep(50);
        can->receive(motor->can_rx);
        usleep(50);

        rs_decode(can_rx, data_fb, data_motor);

        spdlog::info("motor {} - pos : {}, vel : {}", data_fb.id, data_fb.pos, data_fb.vel);

        if (data_fb.id > 0 && data_fb.id <= data->motor_states.size())
        {
            data->motor_states[data_fb.id - 1]->pos = data_fb.pos;
            data->motor_states[data_fb.id - 1]->vel = data_fb.vel;
            data->motor_states[data_fb.id - 1]->tau = data_fb.tau;
        }
        else
        {
            spdlog::warn("motor id {} out of range", data_fb.id);
        }
    }
}
