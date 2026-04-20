#include "can/bus/dm_can_bus_manager.h"
#include "motor/utils/dm_motor_utils.h"
#include "robot/robot_data.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <unistd.h>

DMCANBusManager::DMCANBusManager()
    : CANBusManager()
{
    spdlog::info("DMCANBusManager init");
    type = CanType::DM;
}

// I use special master id for each motor: mst_id = id + 0x40, so i can decode it from can id

void DMCANBusManager::writeState(uint16_t /*index*/, const dm_motor_fb_t& data_fb)
{
    uint16_t id = static_cast<uint16_t>(data_fb.mst_id - 1 - 0x40);

    if (id < data->motor_states.size())
    {
        const auto motor_it = motor_index_map.find(id + 1);
        if (motor_it == motor_index_map.end())
        {
            spdlog::warn("DMCANBusManager received unmapped motor id {}", id + 1);
            return;
        }

        const uint16_t motor_idx = motor_it->second;

        data->motors[motor_idx]->state.pos = data_fb.pos;
        data->motors[motor_idx]->state.vel = data_fb.vel;

        data->motors[motor_idx]->update();

        data->motor_states[id]->pos = data->motors[motor_idx]->state.pos;
        data->motor_states[id]->vel = data->motors[motor_idx]->state.vel;
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

        if (!can->send(motor->can_tx))
        {
            spdlog::warn("Failed to send DM clear_err for motor {}", motor->id);
            continue;
        }
        usleep(50);
        if (!can->receiveMatching(motor->can_rx, [motor](const can_frame& frame) { return dm_is_feedback_frame(frame, motor->id); }))
        {
            spdlog::warn("Timed out waiting for DM clear_err response from motor {}", motor->id);
            continue;
        }
        usleep(50);

        dm_decode(motor->can_rx, data_fb);

        writeState(index, data_fb);
    }

    for (auto index : motor_indices)
    {
        auto motor = data->motors[index];
        auto can   = data->can_interfaces[motor->can_index];

        motor->enable();

        if (!can->send(motor->can_tx))
        {
            spdlog::warn("Failed to send DM enable for motor {}", motor->id);
            continue;
        }
        usleep(50);
        if (!can->receiveMatching(motor->can_rx, [motor](const can_frame& frame) { return dm_is_feedback_frame(frame, motor->id); }))
        {
            spdlog::warn("Timed out waiting for DM enable response from motor {}", motor->id);
            continue;
        }
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

        if (!can->send(motor->can_tx))
        {
            spdlog::warn("Failed to send DM disable for motor {}", motor->id);
            continue;
        }
        usleep(50);
        if (!can->receiveMatching(motor->can_rx, [motor](const can_frame& frame) { return dm_is_feedback_frame(frame, motor->id); }))
        {
            spdlog::warn("Timed out waiting for DM disable response from motor {}", motor->id);
            continue;
        }
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

        cmd->pos = std::clamp(cmd->pos, static_cast<double>(DM_P_MIN),  static_cast<double>(DM_P_MAX));
        cmd->vel = std::clamp(cmd->vel, static_cast<double>(DM_V_MIN),  static_cast<double>(DM_V_MAX));
        cmd->tau = std::clamp(cmd->tau, static_cast<double>(DM_T_MIN),  static_cast<double>(DM_T_MAX));
        cmd->kp  = std::clamp(cmd->kp,  static_cast<double>(DM_KP_MIN), static_cast<double>(DM_KP_MAX));
        cmd->kd  = std::clamp(cmd->kd,  static_cast<double>(DM_KD_MIN), static_cast<double>(DM_KD_MAX));

        motor->setMixedControlInRad(
            cmd->pos, 
            cmd->vel,
            cmd->tau,
            cmd->kp,
            cmd->kd
        );

        if (!can->send(motor->can_tx))
        {
            spdlog::warn("Failed to send DM command for motor {}", motor->id);
            continue;
        }
        usleep(50);
        if (!can->receiveMatching(motor->can_rx, [motor](const can_frame& frame) { return dm_is_feedback_frame(frame, motor->id); }))
        {
            spdlog::warn("Timed out waiting for DM feedback from motor {}", motor->id);
            continue;
        }

        dm_decode(motor->can_rx, data_fb);

        writeState(index, data_fb);
    }
}
