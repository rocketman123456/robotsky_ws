#include "can/bus/rs_can_bus_manager.h"
#include "robot/robot_data.h"

#include <spdlog/spdlog.h>

#include <unistd.h> // usleep

RSCANBusManager::RSCANBusManager()
    : CANBusManager()
{
    spdlog::info("RSCANBusManager init");
    type = CanType::RS;
}

void RSCANBusManager::writeState(uint16_t index, const rs_motor_fb_t& data_fb, const rs_data_read_write& data_motor)
{
    if (data_fb.id > 0 && data_fb.id <= data->motor_states.size())
    {
        uint16_t index = motor_index_map[data_fb.id];

        data->motors[index]->state.pos = data_fb.pos;
        data->motors[index]->state.vel = data_fb.vel;
        data->motors[index]->state.tau = data_fb.tau;

        data->motors[index]->update();

        data->motor_states[data_fb.id - 1]->pos = data->motors[index]->state.pos;
        data->motor_states[data_fb.id - 1]->vel = data->motors[index]->state.vel;
        data->motor_states[data_fb.id - 1]->tau = data->motors[index]->state.tau;
    
        spdlog::info("motor {} - {} - pos : {}, vel : {}", 
            index,
            data_fb.id,
            data->motor_states[data_fb.id - 1]->pos,
            data->motor_states[data_fb.id - 1]->vel
        );
    }
    else
    {
        spdlog::warn("RSCANBusManager motor id {} out of range", data_fb.id);
    }
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

        // spdlog::info("motor {} - pos : {}, vel : {}", data_fb.id, data_fb.pos, data_fb.vel);

        writeState(index, data_fb, data_motor);
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

        // spdlog::info("motor {} - pos : {}, vel : {}", data_fb.id, data_fb.pos, data_fb.vel);

        writeState(index, data_fb, data_motor);
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
        auto cmd   = data->motor_cmds[index];

        motor->setMixedControlInRad(0.0, 0.0, 0.0, 0.0, 1.0);

        can->send(motor->can_tx);
        usleep(50);
        can->receive(motor->can_rx);
        // usleep(50);

        rs_decode(can_rx, data_fb, data_motor);

        // spdlog::info("{} motor {} - pos : {}, vel : {}", index, data_fb.id, data_fb.pos, data_fb.vel);

        writeState(index, data_fb, data_motor);
    }
}
