// CANBusManager.cpp
#include "can/can_bus_manager.h"

#include <spdlog/spdlog.h>

#include <chrono>
#include <iostream>
#include <memory>

// CANBusManager::CANBusManager(const std::string& can_name)
//     : running(false)
// {
//     CanInitInfo info(can_name);
//     can_interface = std::make_shared<CANInterface>();
//     can_interface->initialize(info);
// }

CANBusManager::~CANBusManager()
{
    stop();
    // can_interface->finalize();
}

void CANBusManager::addCAN(std::shared_ptr<CANInterface> can) { can_interface = can; }

void CANBusManager::addMotor(std::shared_ptr<MotorControl> motor)
{
    // TODO : check motor's can index
    motors.push_back(motor);
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
    //
}

void updateState(std::shared_ptr<MotorState> cmd)
{
    //
}

void CANBusManager::run()
{
    using namespace std::chrono;
    double frequencyHz = 500;
    auto   interval    = duration<double>(1.0 / frequencyHz);
    auto   next_time   = steady_clock::now() + interval;

    while (running.load())
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

        // 可以适当sleep一下
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // 等待直到下一个时间点
        std::this_thread::sleep_until(next_time);
        next_time += interval;
    }
}
