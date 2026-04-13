#include "can/bus/dm_can_bus_manager.h"
#include "motor/utils/dm_motor_utils.h"
#include "robot/robot_data.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <unistd.h>

namespace
{
    constexpr int kMaxReceiveAttempts = 8;
    constexpr int kMaxDrainFrames     = 32;

    void mark_state_timeout(const std::shared_ptr<RobotData>& data, uint16_t state_index)
    {
        if (data == nullptr || state_index >= data->motor_states.size())
        {
            return;
        }

        auto&                       state = data->motor_states[state_index];
        std::lock_guard<std::mutex> lock(state->mutex);
        state->health = MotorHealth::TIMEOUT;
    }

    bool decode_dm_feedback_frame(const can_frame& frame, dm_motor_fb_t& data_fb, uint16_t& state_index)
    {
        dm_decode(frame, data_fb);
        if (data_fb.mst_id <= DM_MASTER_ID)
        {
            return false;
        }

        state_index = static_cast<uint16_t>(data_fb.mst_id - 1 - DM_MASTER_ID);
        return true;
    }
} // namespace

DMCANBusManager::DMCANBusManager()
    : CANBusManager()
{
    spdlog::info("DMCANBusManager init");
    type = CanType::DM;
}

void DMCANBusManager::writeState(uint16_t /*index*/, const dm_motor_fb_t& data_fb)
{
    uint16_t id = static_cast<uint16_t>(data_fb.mst_id - 1 - DM_MASTER_ID);

    if (id < data->motor_states.size())
    {
        auto map_it = motor_index_map.find(id + 1);
        if (map_it == motor_index_map.end())
        {
            spdlog::warn("DMCANBusManager missing motor map entry for motor id {}", id + 1);
            return;
        }

        uint16_t motor_idx = map_it->second;

        data->motors[motor_idx]->state.pos = data_fb.pos;
        data->motors[motor_idx]->state.vel = data_fb.vel;

        data->motors[motor_idx]->update();

        auto&                       state = data->motor_states[id];
        std::lock_guard<std::mutex> lock(state->mutex);
        state->pos          = data->motors[motor_idx]->state.pos;
        state->vel          = data->motors[motor_idx]->state.vel;
        state->health       = MotorHealth::OK;
        state->last_rx_time = std::chrono::steady_clock::now();
    }
}

void DMCANBusManager::drainPendingFeedback(const std::shared_ptr<CANInterface>& can, const char* phase)
{
    can_frame frame {};
    int       drained = 0;

    while (drained < kMaxDrainFrames && can->receive(frame))
    {
        dm_motor_fb_t data_fb {};
        uint16_t      state_index = 0;
        if (!decode_dm_feedback_frame(frame, data_fb, state_index))
        {
            spdlog::debug("DM drain {} ignored malformed frame can_id=0x{:X}", phase, frame.can_id);
            continue;
        }

        if (state_index >= data->motor_states.size())
        {
            spdlog::debug("DM drain {} ignored out-of-range feedback mst_id=0x{:X}", phase, data_fb.mst_id);
            continue;
        }

        writeState(0, data_fb);
        ++drained;
    }
}

bool DMCANBusManager::waitForMotorFeedback(const std::shared_ptr<CANInterface>& can, const std::shared_ptr<MotorControl>& motor, const char* phase)
{
    can_frame frame {};
    int       processed = 0;

    for (int attempt = 0; attempt < kMaxReceiveAttempts; ++attempt)
    {
        if (!can->receive(frame))
        {
            usleep(50);
            continue;
        }

        dm_motor_fb_t data_fb {};
        uint16_t      state_index = 0;
        if (!decode_dm_feedback_frame(frame, data_fb, state_index))
        {
            spdlog::debug("DM {} motor {} ignored malformed frame during {}", motor->can_index, motor->id, phase);
            continue;
        }

        if (state_index >= data->motor_states.size())
        {
            spdlog::debug("DM {} motor {} ignored out-of-range feedback mst_id=0x{:X} during {}", motor->can_index, motor->id, data_fb.mst_id, phase);
            continue;
        }

        writeState(0, data_fb);
        ++processed;

        if (data_fb.mst_id == static_cast<int>(motor->id + DM_MASTER_ID))
        {
            return true;
        }
    }

    spdlog::warn("DM can{} motor {} did not observe fresh feedback during {} after processing {} frame(s)", motor->can_index, motor->id, phase, processed);
    return false;
}

void DMCANBusManager::enable()
{
    spdlog::info("DMCANBusManager enable");

    for (auto index : motor_indices)
    {
        auto motor = data->motors[index];
        auto can   = data->can_interfaces[motor->can_index];

        drainPendingFeedback(can, "before_clear_err");
        dm_clear_err(motor->can_tx, motor->id, DM_MIT_MODE);

        if (!can->send(motor->can_tx))
        {
            mark_state_timeout(data, static_cast<uint16_t>(motor->id - 1));
            continue;
        }

        usleep(50);
        if (!waitForMotorFeedback(can, motor, "clear_err"))
        {
            mark_state_timeout(data, static_cast<uint16_t>(motor->id - 1));
        }
    }

    for (auto index : motor_indices)
    {
        auto motor = data->motors[index];
        auto can   = data->can_interfaces[motor->can_index];

        drainPendingFeedback(can, "before_enable");
        motor->enable();

        if (!can->send(motor->can_tx))
        {
            mark_state_timeout(data, static_cast<uint16_t>(motor->id - 1));
            continue;
        }

        usleep(50);
        if (!waitForMotorFeedback(can, motor, "enable"))
        {
            mark_state_timeout(data, static_cast<uint16_t>(motor->id - 1));
        }
    }
}

void DMCANBusManager::disable()
{
    spdlog::info("DMCANBusManager disable");

    for (auto index : motor_indices)
    {
        auto motor = data->motors[index];
        auto can   = data->can_interfaces[motor->can_index];

        drainPendingFeedback(can, "before_disable");
        motor->disable();

        if (!can->send(motor->can_tx))
        {
            mark_state_timeout(data, static_cast<uint16_t>(motor->id - 1));
            continue;
        }

        usleep(50);
        if (!waitForMotorFeedback(can, motor, "disable"))
        {
            mark_state_timeout(data, static_cast<uint16_t>(motor->id - 1));
        }
    }

    spdlog::info("DMCANBusManager disable finish");
}

void DMCANBusManager::step()
{
    for (auto index : motor_indices)
    {
        auto motor = data->motors[index];
        auto can   = data->can_interfaces[motor->can_index];
        auto cmd   = data->motor_cmds[motor->id - 1];

        cmd->pos = std::clamp(cmd->pos, static_cast<double>(DM_P_MIN), static_cast<double>(DM_P_MAX));
        cmd->vel = std::clamp(cmd->vel, static_cast<double>(DM_V_MIN), static_cast<double>(DM_V_MAX));
        cmd->tau = std::clamp(cmd->tau, static_cast<double>(DM_T_MIN), static_cast<double>(DM_T_MAX));
        cmd->kp  = std::clamp(cmd->kp, static_cast<double>(DM_KP_MIN), static_cast<double>(DM_KP_MAX));
        cmd->kd  = std::clamp(cmd->kd, static_cast<double>(DM_KD_MIN), static_cast<double>(DM_KD_MAX));

        drainPendingFeedback(can, "before_step");

        motor->setMixedControlInRad(cmd->pos, cmd->vel, cmd->tau, cmd->kp, cmd->kd);

        if (!can->send(motor->can_tx))
        {
            mark_state_timeout(data, static_cast<uint16_t>(motor->id - 1));
            continue;
        }

        usleep(50);
        if (!waitForMotorFeedback(can, motor, "step"))
        {
            mark_state_timeout(data, static_cast<uint16_t>(motor->id - 1));
        }
    }
}
