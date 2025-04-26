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

void DMCANBusManager::writeState(uint16_t index, const dm_motor_fb_t& data_fb)
{
    uint16_t id = data_fb.mst_id - 1 - 0x40;

    // spdlog::info("motor {} - pos : {}, vel : {}", id + 1, data_fb.pos, data_fb.vel);

    if (id >= 0 && id < data->motor_states.size())
    {
        // TODO : lock

        // spdlog::info("pre-motor {} - pos : {}, vel : {}", 
        //     id + 1,
        //     data_fb.pos,
        //     data_fb.vel
        // );

        uint16_t index = motor_index_map[id + 1];

        data->motors[index]->state.pos = data_fb.pos;
        data->motors[index]->state.vel = data_fb.vel;
        // data->motors[index]->state.tau = data_fb.tau;

        data->motors[index]->update();

        data->motor_states[id]->pos = data->motors[index]->state.pos;
        data->motor_states[id]->vel = data->motors[index]->state.vel;
        // data->motor_states[id]->tau = data->motors[index]->state.tau;

        // spdlog::info("motor {} - {} : state : {}, pos : {}, vel : {}", 
        //     index,
        //     id + 1,
        //     data_fb.state,
        //     data->motor_states[id]->pos,
        //     data->motor_states[id]->vel
        // );
    }
    else
    {
        // spdlog::warn("DMCANBusManager motor id {} out of range", id + 1);
    }
}

void DMCANBusManager::enable()
{
    spdlog::info("DMCANBusManager enable");

    dm_motor_fb_t data_fb;

    for (auto index : motor_indices)
    {
        auto motor = data->motors[index];
        auto can   = data->can_interfaces[motor->can_index];

        dm_clear_err(motor->can_tx, motor->id, DM_MIT_MODE);

        can->send(motor->can_tx);
        usleep(50);
        can->receive(motor->can_rx);
        usleep(50);

        dm_decode(motor->can_rx, data_fb);

        writeState(index, data_fb);
    }

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

        writeState(index, data_fb);
    }
}

void DMCANBusManager::disable()
{
    spdlog::info("DMCANBusManager disable");

    dm_motor_fb_t data_fb;

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

        writeState(index, data_fb);
    }

    spdlog::info("DMCANBusManager disable finish");
}

void DMCANBusManager::step()
{
    dm_motor_fb_t data_fb;

    for (auto index : motor_indices)
    {
        auto motor = data->motors[index];
        auto can   = data->can_interfaces[motor->can_index];
        // auto m_id  = motor_index_map[motor->id];
        auto cmd   = data->motor_cmds[motor->id - 1];

        // clip pos, vel, torque, kp, kd

        if (cmd->pos > DM_P_MAX)
            cmd->pos = DM_P_MAX;
        else if (cmd->pos < DM_P_MIN)
            cmd->pos = DM_P_MIN;
        if (cmd->vel > DM_V_MAX)
            cmd->vel = DM_V_MAX;
        else if (cmd->vel < DM_V_MIN)
            cmd->vel = DM_V_MIN;
        if (cmd->tau > DM_T_MAX)
            cmd->tau = DM_T_MAX;
        else if (cmd->tau < DM_T_MIN)
            cmd->tau = DM_T_MIN;
        if (cmd->kp > DM_KP_MAX)
            cmd->kp = DM_KP_MAX;
        else if (cmd->kp < DM_KP_MIN)
            cmd->kp = DM_KP_MIN;
        if (cmd->kd > DM_KD_MAX)
            cmd->kd = DM_KD_MAX;
        else if (cmd->kd < DM_KD_MIN)
            cmd->kd = DM_KD_MIN;

        motor->setMixedControlInRad(
            cmd->pos, 
            cmd->vel,
            cmd->tau,
            cmd->kp,
            cmd->kd
        );

        can->send(motor->can_tx);
        usleep(50);
        can->receive(motor->can_rx);
        // usleep(50);

        dm_decode(motor->can_rx, data_fb);

        writeState(index, data_fb);
    }
}
