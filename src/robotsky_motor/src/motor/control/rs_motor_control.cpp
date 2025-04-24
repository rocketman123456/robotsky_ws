#include "motor/control/rs_motor_control.h"
#include "motor/utils/rs_motor_utils.h"

void RSMotorControl::setZero() { rs_save_pos_zero(can_tx, id); }

void RSMotorControl::enable()
{
    rs_enable_motor_mode(can_tx, id);
    // rs_set_motor_parameter(can_tx, id, 0X7005, RS_Move_Control_mode, RS_Set_mode);
}

void RSMotorControl::disable() { rs_disable_motor_mode(can_tx, id, 0); }

// void RSMotorControl::setMITMode()
// {
//     rs_set_motor_parameter(can_tx, id, 0X7005, RS_Move_Control_mode, RS_Set_mode);
// }

void RSMotorControl::setMixedControlInDeg(double pos, double vel, double tau, double kp, double kd)
{
    forwardDataComputeDeg(pos, vel, tau, kp, kd);

    rs_mit_ctrl(can_tx, id, cmd.pos, cmd.vel, cmd.kp, cmd.kd, cmd.tau);
}

void RSMotorControl::setMixedControlInRad(double pos, double vel, double tau, double kp, double kd)
{
    forwardDataComputeRad(pos, vel, tau, kp, kd);

    rs_mit_ctrl(can_tx, id, cmd.pos, cmd.vel, cmd.kp, cmd.kd, cmd.tau);
}
