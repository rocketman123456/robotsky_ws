#include "can/bus/rs_can_bus_manager.h"
#include "motor/control/rs_motor_control.h"
#include "robot/robot_data.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <unistd.h>

namespace
{
    constexpr int kMaxReceiveAttempts = 8;
    constexpr int kMaxDrainFrames     = 32;
    constexpr int kFeedbackWaitUs     = 100;

    void mark_state_timeout(const std::shared_ptr<RobotData>& data, uint16_t state_index)
    {
        if (data == nullptr || state_index >= data->motor_states.size() || !data->motor_states[state_index])
        {
            return;
        }

        auto&                       state = data->motor_states[state_index];
        std::lock_guard<std::mutex> lock(state->mutex);
        state->health = MotorHealth::TIMEOUT;
    }

    bool frame_is_rs_motor_request(const can_frame& frame) { return static_cast<uint32_t>((frame.can_id & 0x3F000000) >> 24) == RS_MotorRequest; }

    bool decode_rs_feedback_frame(const can_frame& frame, rs_motor_fb_t& data_fb, rs_data_read_write& data_motor)
    {
        rs_decode(frame, data_fb, data_motor);
        return frame_is_rs_motor_request(frame);
    }
} // namespace

RSCANBusManager::RSCANBusManager()
    : CANBusManager()
{
    spdlog::info("RSCANBusManager init");
    type = CanType::RS;
}

void RSCANBusManager::writeState(uint16_t /*index*/, const rs_motor_fb_t& data_fb, const rs_data_read_write& /*data_motor*/)
{
    if (data_fb.id > 0 && data_fb.id <= data->motor_states.size() && data->motor_states[data_fb.id - 1])
    {
        auto map_it = motor_index_map.find(data_fb.id);
        if (map_it == motor_index_map.end())
        {
            spdlog::warn("RSCANBusManager missing motor map entry for motor id {}", data_fb.id);
            return;
        }

        const uint16_t motor_idx = map_it->second;

        data->motors[motor_idx]->state.pos = data_fb.pos;
        data->motors[motor_idx]->state.vel = data_fb.vel;
        data->motors[motor_idx]->state.tau = data_fb.tau;

        data->motors[motor_idx]->update();

        auto&                       state = data->motor_states[data_fb.id - 1];
        std::lock_guard<std::mutex> lock(state->mutex);
        state->pos          = data->motors[motor_idx]->state.pos;
        state->vel          = data->motors[motor_idx]->state.vel;
        state->tau          = data->motors[motor_idx]->state.tau;
        state->health       = MotorHealth::OK;
        state->last_rx_time = std::chrono::steady_clock::now();
    }
    else
    {
        spdlog::debug("RSCANBusManager ignored out-of-range motor id {}", data_fb.id);
    }
}

void RSCANBusManager::drainPendingFeedback(const std::shared_ptr<CANInterface>& can, const char* phase)
{
    can_frame frame {};
    int       drained = 0;

    while (drained < kMaxDrainFrames && can->isDataAvailable())
    {
        if (!can->receive(frame))
        {
            break;
        }

        rs_motor_fb_t      data_fb {};
        rs_data_read_write data_motor {};
        if (!decode_rs_feedback_frame(frame, data_fb, data_motor))
        {
            spdlog::debug("RS drain {} ignored non-state frame can_id=0x{:X}", phase, frame.can_id);
            continue;
        }

        writeState(0, data_fb, data_motor);
        ++drained;
    }
}

bool RSCANBusManager::waitForMotorFeedback(const std::shared_ptr<CANInterface>& can, const std::shared_ptr<MotorControl>& motor, const char* phase)
{
    can_frame frame {};
    int       processed = 0;

    for (int attempt = 0; attempt < kMaxReceiveAttempts; ++attempt)
    {
        if (!can->isDataAvailable(kFeedbackWaitUs))
        {
            usleep(50);
            continue;
        }

        if (!can->receive(frame))
        {
            continue;
        }

        rs_motor_fb_t      data_fb {};
        rs_data_read_write data_motor {};
        if (!decode_rs_feedback_frame(frame, data_fb, data_motor))
        {
            spdlog::debug("RS {} motor {} ignored non-state frame during {}", motor->can_index, motor->id, phase);
            continue;
        }

        writeState(0, data_fb, data_motor);
        ++processed;

        if (data_fb.id == motor->id)
        {
            return true;
        }
    }

    spdlog::warn("RS can{} motor {} did not observe fresh feedback during {} after processing {} frame(s)", motor->can_index, motor->id, phase, processed);
    return false;
}

void RSCANBusManager::enable()
{
    spdlog::info("RSCANBusManager enable");

    for (auto index : motor_indices)
    {
        auto motor = data->motors[index];
        auto can   = data->can_interfaces[motor->can_index];

        drainPendingFeedback(can, "before_set_mode");
        rs_set_motor_parameter(motor->can_tx, motor->id, 0X7005, RS_Move_Control_mode, RS_Set_mode);

        if (!can->send(motor->can_tx))
        {
            mark_state_timeout(data, static_cast<uint16_t>(motor->id - 1));
            continue;
        }

        usleep(50);
        if (!waitForMotorFeedback(can, motor, "set_mode"))
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

    spdlog::info("RSCANBusManager enable finish");
}

void RSCANBusManager::disable()
{
    spdlog::info("RSCANBusManager disable");

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

    spdlog::info("RSCANBusManager disable finish");
}

void RSCANBusManager::step()
{
    for (auto index : motor_indices)
    {
        auto motor = data->motors[index];
        auto can   = data->can_interfaces[motor->can_index];
        auto cmd   = data->motor_cmds[motor->id - 1];

        cmd->pos = std::clamp(cmd->pos, static_cast<double>(RS_P_MIN), static_cast<double>(RS_P_MAX));
        cmd->vel = std::clamp(cmd->vel, static_cast<double>(RS_V_MIN), static_cast<double>(RS_V_MAX));
        cmd->tau = std::clamp(cmd->tau, static_cast<double>(RS_T_MIN), static_cast<double>(RS_T_MAX));
        cmd->kp  = std::clamp(cmd->kp, static_cast<double>(RS_KP_MIN), static_cast<double>(RS_KP_MAX));
        cmd->kd  = std::clamp(cmd->kd, static_cast<double>(RS_KD_MIN), static_cast<double>(RS_KD_MAX));

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
