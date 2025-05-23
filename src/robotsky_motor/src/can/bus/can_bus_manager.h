#pragma once

#include "can/can_interface.h"
#include "motor/control/motor_control.h"
#include "utils/fps_counter.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>
#include <vector>

struct RobotData;

class CANBusManager
{
public:
    // 构造函数：绑定到某个CAN接口，并关联该接口上的电机列表
    CANBusManager()          = default;
    virtual ~CANBusManager() = default;

    void initialize(const CanBusInitInfo& info);
    void setRobotData(std::shared_ptr<RobotData> data);

    // 启动专用线程，处理CAN总线的收发和数据更新
    void start();
    void stop();

    virtual void enable() = 0;
    virtual void disable() = 0;

    void updateCmd(std::shared_ptr<MotorCmd> cmd);
    void updateState(std::shared_ptr<MotorState> state);

    // 专用线程函数
    virtual void step();
    virtual void run();

    std::shared_ptr<RobotData> data;

    CanType type = CanType::NONE;

    uint16_t cpu_core = 0;

    std::vector<uint16_t> can_indices;
    std::vector<uint16_t> motor_indices;

    std::unordered_map<uint16_t, uint16_t> motor_index_map;

    using Clock     = std::chrono::high_resolution_clock; // steady_clock
    using Duration  = std::chrono::duration<double>;
    using TimePoint = std::chrono::time_point<Clock, Duration>;

    double    frequency_hz = 500;
    Duration  interval     = Duration(1.0 / frequency_hz);
    TimePoint next_time    = Clock::now() + interval;

    // 用于停止线程
    std::atomic<bool> running;
    std::thread       worker_thread;

    FPSCounter counter;
};
