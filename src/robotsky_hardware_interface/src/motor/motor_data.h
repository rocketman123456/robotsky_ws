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
    TORQUE,
};

enum class MotorHealth : uint16_t
{
    OK,
    TIMEOUT,
};

enum class MotorType : uint16_t
{
    DM,
    RS,
    CYBER,
    LK,
    NONE,
};

struct MotorState
{
    std::mutex mutex; // 用于保护写入

    double pos = 0.0;
    double vel = 0.0;
    double tau = 0.0;

    MotorMode   mode   = MotorMode::OFF;
    MotorHealth health = MotorHealth::OK;

    std::chrono::steady_clock::time_point last_rx_time;
};

struct MotorCmd
{
    std::mutex mutex; // 用于保护写入

    double pos = 0.0;
    double vel = 0.0;
    double kp  = 0.0;
    double kd  = 0.0;
    double tau = 0.0;

    MotorMode   mode   = MotorMode::OFF;
    MotorHealth health = MotorHealth::OK;

    std::chrono::steady_clock::time_point last_tx_time;
};

struct MotorInitInfo
{
    MotorType type = MotorType::NONE;
    MotorMode mode = MotorMode::OFF;

    uint16_t can_index  = 0;
    uint16_t id         = 0;
    double   direction  = 1.0;
    double   offset     = 0.0;
    double   delta      = 0.0;
    double   pos_scalar = 1.0;
    double   vel_scalar = 1.0;
    double   tau_scalar = 1.0;
};

// #pragma pack()
