// CANBusManager.cpp
#include "can/can_bus_manager.h"

#include <iostream>

CANInterface::CANInterface(const std::string& can_name_)
    : can_name(can_name_)
    , socket_fd(-1)
{
    // 创建socket并初始化
}

CANInterface::~CANInterface()
{
    // 关闭socket
}

bool CANInterface::initialize()
{
    // 初始化CAN接口（调用socket(), bind(), setsockopt()等）
    return true;
}

bool CANInterface::sendFrame(const std::vector<uint8_t>& frameData)
{
    // 使用write()发送数据
    return true;
}

bool CANInterface::readFrame(std::vector<uint8_t>& frameData)
{
    // 使用read()接收数据
    return true;
}

// CANBusManager
CANBusManager::CANBusManager(const std::string& can_name)
    : can_interface(can_name)
    , running(false)
{
    can_interface.initialize();
}

CANBusManager::~CANBusManager() { stop(); }

void CANBusManager::addMotor(std::shared_ptr<MotorControl> motor) { motors.push_back(motor); }

void CANBusManager::start()
{
    running       = true;
    worker_thread = std::thread(&CANBusManager::run, this);
}

void CANBusManager::stop()
{
    running = false;
    send_cv.notify_all();
    if (worker_thread.joinable())
        worker_thread.join();
}

void CANBusManager::queueSendCommand(const std::vector<uint8_t>& frameData)
{
    {
        std::lock_guard<std::mutex> lock(send_mutex);
        send_queue.push(frameData);
    }
    send_cv.notify_one();
}

void CANBusManager::run()
{
    while (running)
    {
        // 1. 处理待发送命令队列
        std::vector<uint8_t> frame;
        {
            std::unique_lock<std::mutex> lock(send_mutex);
            if (send_queue.empty())
            {
                // 等待新命令或超时检查接收
                send_cv.wait_for(lock, std::chrono::milliseconds(10));
            }
            if (!send_queue.empty())
            {
                frame = send_queue.front();
                send_queue.pop();
            }
        }
        if (!frame.empty())
        {
            can_interface.sendFrame(frame);
        }
        // 2. 读取CAN总线上的数据帧
        std::vector<uint8_t> receivedFrame;
        if (can_interface.readFrame(receivedFrame))
        {
            // 根据CAN帧内容解析，并更新对应电机状态
            // 例如，遍历 motors_ 更新状态（此处可能需要进一步映射ID与电机）
            for (auto& motor : motors)
            {
                // motor->updateState(receivedFrame); // 具体解析逻辑由各电机实现
            }
        }
        // 可以适当sleep一下
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
