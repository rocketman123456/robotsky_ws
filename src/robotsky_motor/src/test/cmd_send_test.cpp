#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <srobot_msg/msg/motor_cmds.hpp>
#include <srobot_msg/msg/motor_states.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

// Callback function that gets called when a message is received
void topic_callback(const std_msgs::msg::String::SharedPtr msg) { RCLCPP_INFO(rclcpp::get_logger("minimal_subscriber"), "I heard: '%s'", msg->data.c_str()); }

int main(int argc, char* argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    srobot_msg::msg::MotorCmds joint_cmd;
    joint_cmd.cmds.resize(16);

    joint_cmd.cmds[3].tau = 0;
    joint_cmd.cmds[3].kp  = 0;
    joint_cmd.cmds[3].kd  = 0;
    joint_cmd.cmds[3].q   = 0.1;
    joint_cmd.cmds[3].dq  = 0;

    // Create a node instance (without creating a custom class)
    auto node = rclcpp::Node::make_shared("motor_cmd_send_test");

    // Create a publisher for the "std_msgs/msg/String" topic with a queue size of 10
    auto publisher = node->create_publisher<srobot_msg::msg::MotorCmds>("/motor_cmds", 10);

    publisher->publish(joint_cmd);

    // Spin the node so that callbacks are processed
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
