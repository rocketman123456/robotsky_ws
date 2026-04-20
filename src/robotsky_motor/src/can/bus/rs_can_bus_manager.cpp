#include "can/bus/rs_can_bus_manager.h"
#include "robot/robot_data.h"
#include "motor/control/rs_motor_control.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <unistd.h>

RSCANBusManager::RSCANBusManager()
    : CANBusManager()
{
    spdlog::info("RSCANBusManager init");
    type = CanType::RS;
}

void RSCANBusManager::writeState(uint16_t /*index*/, const rs_motor_fb_t& data_fb, const rs_data_read_write& /*data_motor*/)
{
    if (data_fb.id > 0 && data_fb.id <= data->motor_states.size())
    {
        const auto motor_it = motor_index_map.find(data_fb.id);
        if (motor_it == motor_index_map.end())
        {
            spdlog::warn("RSCANBusManager received unmapped motor id {}", data_fb.id);
            return;
        }

        const uint16_t motor_idx = motor_it->second;

        data->motors[motor_idx]->state.pos = data_fb.pos;
        data->motors[motor_idx]->state.vel = data_fb.vel;
        data->motors[motor_idx]->state.tau = data_fb.tau;

        data->motors[motor_idx]->update();

        data->motor_states[data_fb.id - 1]->pos = data->motors[motor_idx]->state.pos;
        data->motor_states[data_fb.id - 1]->vel = data->motors[motor_idx]->state.vel;
        data->motor_states[data_fb.id - 1]->tau = data->motors[motor_idx]->state.tau;
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

        if (!can->send(motor->can_tx))
        {
            spdlog::warn("Failed to send RS mode command for motor {}", motor->id);
            continue;
        }
        usleep(50);
        if (!can->receiveMatching(motor->can_rx, [motor](const can_frame& frame) { return rs_is_feedback_frame(frame, motor->id); }))
        {
            spdlog::warn("Timed out waiting for RS mode response from motor {}", motor->id);
            continue;
        }
        usleep(50);

        data_fb    = rs_motor_fb_t{};
        data_motor = rs_data_read_write{};
        rs_decode(motor->can_rx, data_fb, data_motor);

        // spdlog::info("motor {} - pos : {}, vel : {}", data_fb.id, data_fb.pos, data_fb.vel);

        if (data_fb.id != 0)
        {
            writeState(index, data_fb, data_motor);
        }
    }

    for(auto index : motor_indices)
    {
        auto motor = data->motors[index];
        auto can = data->can_interfaces[motor->can_index];

        motor->enable();

        if (!can->send(motor->can_tx))
        {
            spdlog::warn("Failed to send RS enable for motor {}", motor->id);
            continue;
        }
        usleep(50);
        if (!can->receiveMatching(motor->can_rx, [motor](const can_frame& frame) { return rs_is_feedback_frame(frame, motor->id); }))
        {
            spdlog::warn("Timed out waiting for RS enable response from motor {}", motor->id);
            continue;
        }
        usleep(50);

        data_fb    = rs_motor_fb_t{};
        data_motor = rs_data_read_write{};
        rs_decode(motor->can_rx, data_fb, data_motor);

        // spdlog::info("motor {} - pos : {}, vel : {}", data_fb.id, data_fb.pos, data_fb.vel);

        if (data_fb.id != 0)
        {
            writeState(index, data_fb, data_motor);
        }
    }

    spdlog::info("RSCANBusManager enable finish");
}

void RSCANBusManager::disable()
{
    spdlog::info("RSCANBusManager disable");

    rs_motor_fb_t      data_fb;
    rs_data_read_write data_motor;

    for(auto index : motor_indices)
    {
        auto motor = data->motors[index];
        auto can = data->can_interfaces[motor->can_index];

        motor->disable();

        if (!can->send(motor->can_tx))
        {
            spdlog::warn("Failed to send RS disable for motor {}", motor->id);
            continue;
        }
        usleep(50);
        if (!can->receiveMatching(motor->can_rx, [motor](const can_frame& frame) { return rs_is_feedback_frame(frame, motor->id); }))
        {
            spdlog::warn("Timed out waiting for RS disable response from motor {}", motor->id);
            continue;
        }
        usleep(50);

        data_fb    = rs_motor_fb_t{};
        data_motor = rs_data_read_write{};
        rs_decode(motor->can_rx, data_fb, data_motor);

        // spdlog::info("motor {} - pos : {}, vel : {}", data_fb.id, data_fb.pos, data_fb.vel);

        if (data_fb.id != 0)
        {
            writeState(index, data_fb, data_motor);
        }
    }

    spdlog::info("RSCANBusManager disable finish");
}

void RSCANBusManager::step()
{
    rs_motor_fb_t      data_fb;
    rs_data_read_write data_motor;

    for(auto index : motor_indices)
    {
        auto motor = data->motors[index];
        auto can   = data->can_interfaces[motor->can_index];
        auto cmd   = data->motor_cmds[motor->id - 1];

        cmd->pos = std::clamp(cmd->pos, static_cast<double>(RS_P_MIN),  static_cast<double>(RS_P_MAX));
        cmd->vel = std::clamp(cmd->vel, static_cast<double>(RS_V_MIN),  static_cast<double>(RS_V_MAX));
        cmd->tau = std::clamp(cmd->tau, static_cast<double>(RS_T_MIN),  static_cast<double>(RS_T_MAX));
        cmd->kp  = std::clamp(cmd->kp,  static_cast<double>(RS_KP_MIN), static_cast<double>(RS_KP_MAX));
        cmd->kd  = std::clamp(cmd->kd,  static_cast<double>(RS_KD_MIN), static_cast<double>(RS_KD_MAX));

        motor->setMixedControlInRad(
            cmd->pos, 
            cmd->vel,
            cmd->tau,
            cmd->kp,
            cmd->kd
        );

        if (!can->send(motor->can_tx))
        {
            spdlog::warn("Failed to send RS command for motor {}", motor->id);
            continue;
        }
        usleep(50);
        if (!can->receiveMatching(motor->can_rx, [motor](const can_frame& frame) { return rs_is_feedback_frame(frame, motor->id); }))
        {
            spdlog::warn("Timed out waiting for RS feedback from motor {}", motor->id);
            continue;
        }

        data_fb    = rs_motor_fb_t{};
        data_motor = rs_data_read_write{};
        rs_decode(motor->can_rx, data_fb, data_motor);

        // spdlog::info("{} motor {} - pos : {}, vel : {}", index, data_fb.id, data_fb.pos, data_fb.vel);

        if (data_fb.id != 0)
        {
            writeState(index, data_fb, data_motor);
        }
        else
        {
            spdlog::warn("RS feedback decode returned empty motor id for commanded motor {}", motor->id);
        }
    }
}
