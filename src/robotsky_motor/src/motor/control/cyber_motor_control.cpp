#include "motor/control/cyber_motor_control.h"
#include "motor/utils/cyber_gear_utils.h"

void CyberMotorControl::setZero() { cyber_set_zero(id, can_tx); }

void CyberMotorControl::enable() { cyber_enable(id, can_tx); }

void CyberMotorControl::disable() { cyber_disable(id, can_tx); }

void CyberMotorControl::setMixedControlInDeg(double pos, double vel, double tau, double kp, double kd)
{
    forwardDataComputeDeg(pos, vel, tau, kp, kd);

    cyber_mixed_control(id, cmd.pos, cmd.vel, cmd.kp, cmd.kd, cmd.tau, can_tx);
}

void CyberMotorControl::setMixedControlInRad(double pos, double vel, double tau, double kp, double kd)
{
    forwardDataComputeRad(pos, vel, tau, kp, kd);

    cyber_mixed_control(id, cmd.pos, cmd.vel, cmd.kp, cmd.kd, cmd.tau, can_tx);
}

// MotorState RSMotorControl::decode()
// {
//     dm_motor_fb_t data;
//
//     dm_decode(can_rx, data);
//
//     MotorState state;
//
//     state.pos = data.pos;
//     state.vel = data.vel;
//     state.tau = data.tau;
//
//     state.last_rx_time = std::chrono::steady_clock::now();
//
//     return state;
// }