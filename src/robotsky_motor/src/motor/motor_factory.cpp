#include "motor/motor_factory.h"

#include "motor/cyber_motor_control.h"
#include "motor/dm_motor_control.h"
#include "motor/rs_motor_control.h"

std::shared_ptr<MotorControl> create_motor_control(const MotorInitInfo& info)
{
    switch (info.type)
    {
        case MotorType::DM:
            return std::make_shared<DMMotorControl>();
        case MotorType::RS:
            return std::make_shared<RSMotorControl>();
        case MotorType::CYBER:
            return std::make_shared<CyberMotorControl>();
            return {};
        case MotorType::LK:
            // return std::make_shared<LKMotorControl>(info);
            return {};
        default:
            return {};
    }
}