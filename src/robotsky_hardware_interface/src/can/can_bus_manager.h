#pragma once

#include "can/can_driver.h"
#include "motor/motor_control.h"

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

// 假设CAN接口相关函数封装在一个CANInterface类中
class CANInterface
{
public:
    explicit CANInterface(const std::string& canName);
    ~CANInterface();
    bool initialize();
    bool sendFrame(const std::vector<uint8_t>& frameData);
    bool readFrame(std::vector<uint8_t>& frameData);

    std::string can_name;
    int         socket_fd;
};

class CANBusManager
{
public:
    // 构造函数：绑定到某个CAN接口，并关联该接口上的电机列表
    explicit CANBusManager(const std::string& canName);
    ~CANBusManager();

    // 添加电机到该总线管理器中
    void addMotor(std::shared_ptr<MotorControl> motor);

    // 启动专用线程，处理CAN总线的收发和数据更新
    void start();
    void stop();

    // 外部可以调用发送接口，将待发送数据放入队列中
    void queueSendCommand(const std::vector<uint8_t>& frameData);

private:
    // 专用线程函数
    void run();

public:
    // 内部CAN接口
    CANInterface can_interface;

    // 该总线上的电机集合
    std::vector<std::shared_ptr<MotorControl>> motors;
    // 用于发送命令的队列及同步原语
    std::queue<std::vector<uint8_t>> send_queue;
    std::mutex                       send_mutex;
    std::condition_variable          send_cv;

    // 用于停止线程
    std::atomic<bool> running;
    std::thread       worker_thread;
};
