#pragma once

#include "can/can_bus_manager.h"
#include "can/can_interface.h"
#include "motor/motor_data.h"

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

enum class RobotState : uint16_t
{
    IDLE,
    RUNNING,
    ERROR
};

class Robot
{
public:
    Robot();
    ~Robot();

    void initCAN(std::vector<CanInitInfo>& can_infos);
    void initMotors(std::vector<MotorInitInfo>& motor_infos);

    void start();
    void stop();

    void updateExternalCommand(int motorId, MotorMode mode);                // 外部控制接口
    void tickStateMachine();                                                // 运行状态机
    void updateFromCAN(int motorId, double pos, double vel, double torque); // CAN线程调用

    void checkStateTimeouts(); // 定期检查电机超时
    void checkCmdTimeouts();

    std::vector<std::shared_ptr<MotorState>> motor_states;
    std::vector<std::shared_ptr<MotorCmd>>   motor_cmds;

    // 状态机状态（简化）
    RobotState state = RobotState::IDLE;

    std::mutex state_mutex;
};