#pragma once

#include "can/can_interface.h"
#include "motor/control/motor_control.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

class CANBusManager
{
public:
    // 构造函数：绑定到某个CAN接口，并关联该接口上的电机列表
    CANBusManager()          = default;
    virtual ~CANBusManager() = default;

    void addCAN(std::shared_ptr<CANInterface> can);
    void addMotor(std::shared_ptr<MotorControl> motor);

    // 启动专用线程，处理CAN总线的收发和数据更新
    void start();
    void stop();

    // 外部可以调用发送接口，将待发送数据放入队列中
    // void queueSendCommand(const std::vector<uint8_t>& frameData);

    void updateCmd(std::shared_ptr<MotorCmd> cmd);
    void updateState(std::shared_ptr<MotorState> cmd);

private:
    // 专用线程函数
    virtual void step();
    virtual void run();

public:
    // 内部CAN接口
    std::shared_ptr<CANInterface> can_interface;

    // 该总线上的电机集合
    std::vector<std::shared_ptr<MotorControl>> motors;

    using Clock     = std::chrono::high_resolution_clock; // steady_clock
    using Duration  = std::chrono::duration<double>;
    using TimePoint = std::chrono::time_point<Clock, Duration>;

    double    frequency_hz = 500;
    Duration  interval     = Duration(1.0 / frequency_hz);
    TimePoint next_time    = Clock::now() + interval;

    // 用于停止线程
    std::atomic<bool> running;
    std::thread       worker_thread;
};
