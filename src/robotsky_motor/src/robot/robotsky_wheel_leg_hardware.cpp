#include "robot/robotsky_wheel_leg_hardware.h"

namespace robotsky
{
namespace wheel_leg
{
std::vector<CanInitInfo> prepareCan()
{
    return {
        {"can0"},
        {"can1"},
        {"can2"},
        {"can3"},
    };
}

std::vector<MotorInitInfo> prepareMotors()
{
    return {
        {MotorType::DM, MotorMode::POSITION, 0, 0x01, +1.0, 0.0, 0.0, 1.0, 1.0, 1.0},
        {MotorType::DM, MotorMode::VELOCITY, 0, 0x04, -1.0, 0.0, 0.0, 1.0, 1.0, 1.0},
        {MotorType::DM, MotorMode::POSITION, 0, 0x05, +1.0, 0.0, 0.0, 1.0, 1.0, 1.0},
        {MotorType::DM, MotorMode::VELOCITY, 0, 0x08, +1.0, 0.0, 0.0, 1.0, 1.0, 1.0},

        {MotorType::DM, MotorMode::POSITION, 1, 0x09, -1.0, 0.0, 0.0, 1.0, 1.0, 1.0},
        {MotorType::DM, MotorMode::VELOCITY, 1, 0x0c, -1.0, 0.0, 0.0, 1.0, 1.0, 1.0}, // 12
        {MotorType::DM, MotorMode::POSITION, 1, 0x0d, -1.0, 0.0, 0.0, 1.0, 1.0, 1.0}, // 13
        {MotorType::DM, MotorMode::VELOCITY, 1, 0x10, +1.0, 0.0, 0.0, 1.0, 1.0, 1.0}, // 16

        {MotorType::RS, MotorMode::POSITION, 2, 0x02, +1.0, 0.0, 0.0, 1.0, 1.0, 1.0},
        {MotorType::RS, MotorMode::POSITION, 2, 0x03, +1.0, 0.0, 0.0, 1.0, 1.0, 1.0},
        {MotorType::RS, MotorMode::POSITION, 2, 0x06, -1.0, 0.0, 0.0, 1.0, 1.0, 1.0},
        {MotorType::RS, MotorMode::POSITION, 2, 0x07, -1.0, 0.0, 0.0, 1.0, 1.0, 1.0},

        {MotorType::RS, MotorMode::POSITION, 3, 0x0a, +1.0, 0.0, 0.0, 1.0, 1.0, 1.0}, // 10
        {MotorType::RS, MotorMode::POSITION, 3, 0x0b, +1.0, 0.0, 0.0, 1.0, 1.0, 1.0}, // 11
        {MotorType::RS, MotorMode::POSITION, 3, 0x0e, -1.0, 0.0, 0.0, 1.0, 1.0, 1.0}, // 14
        {MotorType::RS, MotorMode::POSITION, 3, 0x0f, -1.0, 0.0, 0.0, 1.0, 1.0, 1.0}, // 15
    };
}

std::vector<CanBusInitInfo> prepareCanBuses()
{
    return {
        // {CanType::DM, 1, {0, 1}, {0, 1, 2, 3, 4, 5, 6, 7}},
        // {CanType::RS, 2, {2, 3}, {8, 9, 10, 11, 12, 13, 14, 15}},
        {CanType::DM, 1, {0, 1}, {0, 1, 2, 3, 4, 5, 6, 7}},
        {CanType::RS, 2, {2, 3}, {8, 9, 10, 11, 12, 13, 14, 15}},
    };
}

std::vector<std::string> prepareJointNames()
{
    return {
        "RF_Roll_Joint",
        "RF_Hip_Joint",
        "RF_Knee_Joint",
        "RF_Wheel_Joint",

        "LF_Roll_Joint",
        "LF_Hip_Joint",
        "LF_Knee_Joint",
        "LF_Wheel_Joint",

        "RB_Roll_Joint",
        "RB_Hip_Joint",
        "RB_Knee_Joint",
        "RB_Wheel_Joint",

        "LB_Roll_Joint",
        "LB_Hip_Joint",
        "LB_Knee_Joint",
        "LB_Wheel_Joint",
    };
}

StartupTargets prepareStartupTargets()
{
    StartupTargets targets {};
    targets.pos_begin = {
        0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f,
    };
    targets.pos_hold = {
    //     0.0f, -0.5f,  1.0f, 0.0f,
    //    -0.0f, -0.5f,  1.0f, 0.0f,
    //     0.0f,  0.5f, -1.0f, 0.0f,
    //    -0.0f,  0.5f, -1.0f, 0.0f,
        0.0f, -0.0f,  0.0f, 0.0f,
       -0.0f, -0.0f,  0.0f, 0.0f,
        0.0f,  0.0f, -0.0f, 0.0f,
       -0.0f,  0.0f, -0.0f, 0.0f,
    };
    targets.vel = {
        0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f,
    };
    targets.tau = {
        0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f,
    };
    targets.kp_begin = {
        0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f,
    };
    targets.kp_hold = {
        20.0f, 20.0f, 40.0f, 0.0f,
        20.0f, 20.0f, 40.0f, 0.0f,
        20.0f, 20.0f, 40.0f, 0.0f,
        20.0f, 20.0f, 40.0f, 0.0f,
    };
    targets.kd = {
        1.0f, 1.0f, 1.0f, 1.0f,
        1.0f, 1.0f, 1.0f, 1.0f,
        1.0f, 1.0f, 1.0f, 1.0f,
        1.0f, 1.0f, 1.0f, 1.0f,
    };
    return targets;
}

} // namespace wheel_leg
} // namespace robotsky
