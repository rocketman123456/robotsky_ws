#include "motor/motor_factory.h"

#include "motor/dm_motor_control.h"
#include "motor/rs_motor_control.h"

#include <memory>

std::shared_ptr<MotorControl> create_motor_control(const MotorInitInfo& info)
{
    switch (info.type)
    {
        case MotorType::DM:
            // return std::make_shared<DMotorControl>(info);
            return {};
        case MotorType::RS:
            // return std::make_shared<RSMotorControl>(info);
            return {};
        case MotorType::CYBER:
            // return std::make_shared<CYBERMotorControl>(info);
            return {};
        case MotorType::LK:
            // return std::make_shared<LKMotorControl>(info);
            return {};
        default:
            return {};
    }
}