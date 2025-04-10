//
// Created by Mu Shibo on 12/2024
//

#include <sstream>
#include <memory>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "can_msgs/Frame.h"
#include "motor_control/motor_config.h"
#include "motor_control/MotorFeedback.h"

#include "stdint.h"
#include "math.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_control_test");
    ros::NodeHandle nm;
    ros::Rate loop_rate(300);

    std::vector<uint8_t> left_motor_ids = {0x01, 0x02};

    std::shared_ptr<MotorControlSet> _left_controller = 
        std::make_shared<MotorControlSet>(&nm, "can1_rx", "can1_tx", left_motor_ids);

    ros::Duration(0.5).sleep(); 
    // enable the motor
    for(int i = 0; i < Joint_Num; i++){
        _left_controller->getMotor(i+1).Enable_Motor();
        std::cout << "Joint motor " << i+1 << " is enabled."<< std::endl;
    }
    ros::Duration(0.1).sleep(); 

    while (ros::ok())
    {   
        // _left_controller->getMotor(1).RobStrite_Motor_Pos_control(1,1);
        // _left_controller->getMotor(2).RobStrite_Motor_Pos_control(1,1);
        _left_controller->getMotor(1).RobStrite_Motor_move_control(0, 0, 1, 0, 1);
        _left_controller->getMotor(2).RobStrite_Motor_move_control(0, 0, 0.1, 0, 1);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
