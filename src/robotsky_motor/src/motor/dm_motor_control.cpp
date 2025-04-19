#include "motor/dm_motor_control.h"
#include "motor/utils/dm_motor_utils.h"

void DMMotorControl::setZero() { dm_save_pos_zero(can_tx, id, DM_MIT_MODE); }

void DMMotorControl::enable() { dm_enable_motor_mode(can_tx, id, DM_MIT_MODE); }

void DMMotorControl::disable() { dm_disable_motor_mode(can_tx, id, DM_MIT_MODE); }

void DMMotorControl::setMixedControlInDeg(double pos, double vel, double tau, double kp, double kd)
{
    forwardDataComputeDeg(pos, vel, tau, kp, kd);

    dm_mit_ctrl(can_tx, id, cmd.pos, cmd.vel, cmd.kp, cmd.kd, cmd.tau);
}

void DMMotorControl::setMixedControlInRad(double pos, double vel, double tau, double kp, double kd)
{
    forwardDataComputeRad(pos, vel, tau, kp, kd);

    dm_mit_ctrl(can_tx, id, cmd.pos, cmd.vel, cmd.kp, cmd.kd, cmd.tau);
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