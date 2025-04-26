#include "can/bus/rs_can_bus_manager.h"
#include "robot/robot_data.h"
#include "motor/control/rs_motor_control.h"

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
    
        // spdlog::info("motor {} - {} - pos : {}, vel : {}", 
        //     index,
        //     data_fb.id,
        //     data->motor_states[data_fb.id - 1]->pos,
        //     data->motor_states[data_fb.id - 1]->vel
        // );
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

    for(auto index : motor_indices)
    {
        auto motor = data->motors[index];
        auto can = data->can_interfaces[motor->can_index];

        // auto* pointer = motor.get();
        // reinterpret_cast<RSMotorControl*>(pointer)->setMITMode();

        rs_set_motor_parameter(motor->can_tx, motor->id, 0X7005, RS_Move_Control_mode, RS_Set_mode);

        can->send(motor->can_tx);
        usleep(50);
        can->receive(motor->can_rx);
        usleep(50);

        rs_decode(motor->can_tx, data_fb, data_motor);

        // spdlog::info("motor {} - pos : {}, vel : {}", data_fb.id, data_fb.pos, data_fb.vel);

        writeState(index, data_fb, data_motor);
    }

    for(auto index : motor_indices)
    {
        auto motor = data->motors[index];
        auto can = data->can_interfaces[motor->can_index];

        motor->enable();

        can->send(motor->can_tx);
        usleep(50);
        can->receive(motor->can_rx);
        usleep(50);

        rs_decode(motor->can_tx, data_fb, data_motor);

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

        rs_decode(motor->can_tx, data_fb, data_motor);

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
        // auto m_id  = motor_index_map[motor->id];
        auto cmd   = data->motor_cmds[motor->id - 1];

        if (cmd->pos > RS_P_MAX)
            cmd->pos = RS_P_MAX;
        else if (cmd->pos < RS_P_MIN)
            cmd->pos = RS_P_MIN;
        if (cmd->vel > RS_V_MAX)
            cmd->vel = RS_V_MAX;
        else if (cmd->vel < RS_V_MIN)
            cmd->vel = RS_V_MIN;
        if (cmd->tau > RS_T_MAX)
            cmd->tau = RS_T_MAX;
        else if (cmd->tau < RS_T_MIN)
            cmd->tau = RS_T_MIN;
        if (cmd->kp > RS_KP_MAX)
            cmd->kp = RS_KP_MAX;
        else if (cmd->kp < RS_KP_MIN)
            cmd->kp = RS_KP_MIN;
        if (cmd->kd > RS_KD_MAX)
            cmd->kd = RS_KD_MAX;
        else if (cmd->kd < RS_KD_MIN)
            cmd->kd = RS_KD_MIN;

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

        rs_decode(motor->can_rx, data_fb, data_motor);

        // spdlog::info("{} motor {} - pos : {}, vel : {}", index, data_fb.id, data_fb.pos, data_fb.vel);

        writeState(index, data_fb, data_motor);
    }
}
