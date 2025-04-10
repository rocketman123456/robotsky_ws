#pragma once

#include <cstdint>

// #pragma pack(1)

struct motor_data_t
{
    uint16_t can_id  = 0;
    double   pos     = 0.0;
    double   vel     = 0.0;
    double   tau     = 0.0;
    double   tmp     = 0.0;
    double   pos_des = 0.0;
    double   vel_des = 0.0;
    double   tau_des = 0.0;
    double   kp      = 0.0;
    double   kd      = 0.0;
};

// #pragma pack()
