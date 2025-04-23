#include "motor/control/motor_control.h"

#include <spdlog/spdlog.h>

void MotorControl::initialize(const MotorInitInfo& info)
{
    spdlog::info("Motor init: {}, {}, {}",
        info.can_index,
        info.id,
        info.direction
    );

    type       = info.type;
    can_index  = info.can_index;
    id         = info.id;
    direction  = info.direction;
    offset     = info.offset;
    delta      = info.delta;
    pos_scalar = info.pos_scalar;
    vel_scalar = info.vel_scalar;
    tau_scalar = info.tau_scalar;
}

void MotorControl::forwardDataComputeDeg(double pos, double vel, double tau, double kp, double kd)
{
    // std::lock_guard<std::mutex> lock(cmd.mutex);

    if (tau > torque_upper_limit)
        tau = torque_upper_limit;
    else if (tau < torque_lower_limit)
        tau = torque_lower_limit;

    // add position delta
    cmd.pos = direction * ((pos * M_PI / 180.0 - delta) * pos_scalar) + offset * pos_scalar;
    cmd.vel = vel * direction * vel_scalar * M_PI / 180.0;
    cmd.tau = tau * direction * tau_scalar;
    cmd.kp  = kp;
    cmd.kd  = kd;

    cmd.last_tx_time = std::chrono::steady_clock::now();
}

void MotorControl::forwardDataComputeRad(double pos, double vel, double tau, double kp, double kd)
{
    // std::lock_guard<std::mutex> lock(cmd.mutex);

    if (tau > torque_upper_limit)
        tau = torque_upper_limit;
    else if (tau < torque_lower_limit)
        tau = torque_lower_limit;

    cmd.pos = direction * ((pos - delta) * pos_scalar) + offset;
    cmd.vel = direction * vel * vel_scalar;
    cmd.tau = direction * tau * tau_scalar;
    cmd.kp  = kp;
    cmd.kd  = kd;

    cmd.last_tx_time = std::chrono::steady_clock::now();
}

void MotorControl::update()
{
    // handle data
    // std::lock_guard<std::mutex> lock(state.mutex);

    state.pos += delta;
    state.pos *= direction;
    state.pos /= pos_scalar;
    state.pos += offset;

    state.vel /= vel_scalar;
    state.vel *= direction;

    state.tau /= tau_scalar;
    state.tau *= direction;

    state.last_rx_time = std::chrono::steady_clock::now();
}

// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------

double MotorControl::getPositionRad() const { return state.pos; }
double MotorControl::getVelocityRad() const { return state.vel; }
double MotorControl::getPositionDeg() const { return state.pos * 180.0 / M_PI; }
double MotorControl::getVelocityDeg() const { return state.vel * 180.0 / M_PI; }
double MotorControl::getTorque() const { return state.tau; }
