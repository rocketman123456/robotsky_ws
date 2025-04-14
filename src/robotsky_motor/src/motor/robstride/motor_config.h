//
// Created by Mu Shibo on 04/2024
//

#pragma once

#include <memory>
#include <vector>

#include "ros/ros.h"

#include "can_msgs/Frame.h"

#include "motor_control/MotorFeedback.h"
#include "motor_control/robstride.h"
#include "stdint.h"

#define Joint_Num 2

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -44.0f
#define V_MAX 44.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -17.0f
#define T_MAX 17.0f

typedef struct
{
    uint16_t state;
    float    pos;
    float    vel;
    float    tor;
    float    Kp;
    float    Kd;
    float    Tmos;
    float    Tcoil;
} motor_state_t;

typedef struct
{
    float pos_d;
    float vel_d;
    float tor_d;
    float Kp;
    float Kd;
} motor_cmd_t;

class MotorControlSet
{
public:
    MotorControlSet(ros::NodeHandle* node_handle, const std::string& can_rx, const std::string& can_tx, const std::vector<uint8_t>& motor_ids);
    ~MotorControlSet();

    void shutdownCallback();

    // create subscriber
    ros::Subscriber can_receive;
    ros::Subscriber command_bridge;
    // creat publisher
    ros::Publisher can_send;
    ros::Publisher robstride_state_pub;

    void can1_rx_Callback(can_msgs::Frame msg);
    void command_Callback(can_msgs::Frame msg);

    motor_state_t motor_state[Joint_Num];
    motor_cmd_t   motor_cmd[Joint_Num];

    motor_control::MotorFeedback joint_feedback;

    RobStrite_Motor& getMotor(uint8_t ID)
    {
        if (ID == 0x01)
            return l1_joint;
        else if (ID == 0x02)
            return l2_joint;
    }

private:
    RobStrite_Motor l1_joint;
    RobStrite_Motor l2_joint;
};
