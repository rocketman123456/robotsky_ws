//
// Created by Mu Shibo on 04/2024
//

#include "motor_control/motor_config.h"

MotorControlSet::MotorControlSet(ros::NodeHandle* node_handle, const std::string& can_rx, const std::string& can_tx, const std::vector<uint8_t>& motor_ids)
    : l1_joint(motor_ids[0], node_handle, "robstride_cmd")
    , l2_joint(motor_ids[1], node_handle, "robstride_cmd")
{
    can_receive         = node_handle->subscribe(can_rx, 100, &MotorControlSet::can1_rx_Callback, this);
    can_send            = node_handle->advertise<can_msgs::Frame>(can_tx, 100);
    command_bridge      = node_handle->subscribe("robstride_cmd", 100, &MotorControlSet::command_Callback, this);
    robstride_state_pub = node_handle->advertise<motor_control::MotorFeedback>("/robstride_state", 100);

    joint_feedback.pos.resize(Joint_Num);
    joint_feedback.vel.resize(Joint_Num);
    joint_feedback.tor.resize(Joint_Num);
    joint_feedback.temp.resize(Joint_Num);
    joint_feedback.error_code.resize(Joint_Num);
    joint_feedback.pattern.resize(Joint_Num);
}

void MotorControlSet::shutdownCallback()
{
    l1_joint.Disenable_Motor(0);
    l2_joint.Disenable_Motor(0);
}

MotorControlSet::~MotorControlSet()
{
    shutdownCallback();
    // std::cout << "All joint motor have been disabled." << std::endl;
}

void MotorControlSet::can1_rx_Callback(can_msgs::Frame msg)
{
    if (uint8_t((msg.id & 0xFF00) >> 8) == 0x01)
    {
        l1_joint.RobStrite_Motor_Analysis(msg.data.data(), msg.id);
        joint_feedback.pos[0]        = l1_joint.Pos_Info.Angle;
        joint_feedback.vel[0]        = l1_joint.Pos_Info.Speed;
        joint_feedback.tor[0]        = l1_joint.Pos_Info.Torque;
        joint_feedback.temp[0]       = l1_joint.Pos_Info.Temp;
        joint_feedback.error_code[0] = l1_joint.error_code;
        joint_feedback.pattern[0]    = l1_joint.Pos_Info.pattern;
    }
    else if (uint8_t((msg.id & 0xFF00) >> 8) == 0x02)
    {
        l2_joint.RobStrite_Motor_Analysis(msg.data.data(), msg.id);
        joint_feedback.pos[1]        = l2_joint.Pos_Info.Angle;
        joint_feedback.vel[1]        = l2_joint.Pos_Info.Speed;
        joint_feedback.tor[1]        = l2_joint.Pos_Info.Torque;
        joint_feedback.temp[1]       = l2_joint.Pos_Info.Temp;
        joint_feedback.error_code[1] = l2_joint.error_code;
        joint_feedback.pattern[1]    = l2_joint.Pos_Info.pattern;
    }

    robstride_state_pub.publish(joint_feedback);
}

void MotorControlSet::command_Callback(can_msgs::Frame msg) { can_send.publish(msg); }
