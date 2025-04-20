#include "can/bus/can_bus_manager.h"
#include "robot/robot_data.h"
#include "utils/utils.h"

#include <spdlog/spdlog.h>

#include <cassert>
#include <chrono>
#include <iostream>
#include <memory>

void CANBusManager::initialize(const CanBusInitInfo& info)
{
    type          = info.type;
    cpu_core      = info.cpu_core;
    can_indices   = info.can_indices;
    motor_indices = info.motor_indices;
}

void CANBusManager::setRobotData(std::shared_ptr<RobotData> data)
{
    // set data pointer
    assert(data != nullptr && "robot data pointer is null");
    this->data = data;
}

void CANBusManager::start()
{
    running       = true;
    worker_thread = std::thread(&CANBusManager::run, this);
}

void CANBusManager::stop()
{
    running = false;
    if (worker_thread.joinable())
        worker_thread.join();
}

void updateCmd(std::shared_ptr<MotorCmd> cmd)
{
    assert(cmd != nullptr && "motor cmd pointer is null");
    //
}

void updateState(std::shared_ptr<MotorState> state)
{
    assert(state != nullptr && "motor state pointer is null");
    //
}

void CANBusManager::step()
{
    // std::cout << "Tick at " << duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count() << " ms\n";

    // 1. 处理待发送命令队列
    // std::vector<uint8_t> frame;
    // {
    //     std::unique_lock<std::mutex> lock(send_mutex);
    //     if (send_queue.empty())
    //     {
    //         // 等待新命令或超时检查接收
    //         send_cv.wait_for(lock, std::chrono::milliseconds(10));
    //     }
    //     if (!send_queue.empty())
    //     {
    //         frame = send_queue.front();
    //         send_queue.pop();
    //     }
    // }
    // if (!frame.empty())
    // {
    //     can_interface.sendFrame(frame);
    // }
    // 2. 读取CAN总线上的数据帧
    // std::vector<uint8_t> receivedFrame;
    // if (can_interface.readFrame(receivedFrame))
    // {
    //     // 根据CAN帧内容解析，并更新对应电机状态
    //     // 例如，遍历 motors_ 更新状态（此处可能需要进一步映射ID与电机）
    //     for (auto& motor : motors)
    //     {
    //         // motor->updateState(receivedFrame); // 具体解析逻辑由各电机实现
    //     }
    // }

    // 等待直到下一个时间点
    std::this_thread::sleep_until(next_time);
    next_time += interval;
}

void CANBusManager::run()
{
    set_thread(cpu_core, pthread_self());

    while (running.load())
    {
        step();
    }
}
