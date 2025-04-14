#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>

// #pragma pack(1)

enum class MotorMode : uint16_t
{
    OFF,
    POSITION,
    VELOCITY,
    TORQUE
};

enum class MotorHealth : uint16_t
{
    OK,
    TIMEOUT
};

struct MotorState
{
    std::mutex mutex; // 用于保护写入（也可以换成atomic或锁分离策略）

    double pos = 0.0;
    double vel = 0.0;
    double tau = 0.0;

    MotorMode   mode   = MotorMode::OFF;
    MotorHealth health = MotorHealth::OK;

    std::chrono::steady_clock::time_point last_rx_time;
};

struct MotorCmd
{
    std::mutex mutex; // 用于保护写入（也可以换成atomic或锁分离策略）

    double pos = 0.0;
    double vel = 0.0;
    double kp  = 0.0;
    double kd  = 0.0;
    double tau = 0.0;

    MotorMode   mode   = MotorMode::OFF;
    MotorHealth health = MotorHealth::OK;

    std::chrono::steady_clock::time_point last_tx_time;
};

// #pragma pack()
