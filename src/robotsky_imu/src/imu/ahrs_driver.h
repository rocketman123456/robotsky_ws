#ifndef BASE_DRIVER_H_
#define BASE_DRIVER_H_

#include "imu/fdilink_data_struct.h"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "tf2_ros/transform_broadcaster.h"

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/src/Core/Matrix.h>

// http://wjwwood.io/serial/doc/1.1.0/index.html
#include <serial/serial.h>
#include <string>

namespace FDILink
{
#define FRAME_HEAD 0xfc
#define FRAME_END 0xfd
#define TYPE_IMU 0x40
#define TYPE_AHRS 0x41
#define TYPE_INSGPS 0x42
#define TYPE_GEODETIC_POS 0x5c
#define TYPE_GROUND 0xf0
#define IMU_LEN 0x38          // 56
#define AHRS_LEN 0x30         // 48
#define INSGPS_LEN 0x48       // 72
#define GEODETIC_POS_LEN 0x20 // 32
#define PI 3.141592653589793
#define DEG_TO_RAD 0.017453292519943295

    class AhrsBringup : public rclcpp::Node
    {
    public:
        AhrsBringup();
        ~AhrsBringup();

        void initialize();
        void processLoop();

    private:
        double magCalculateYaw(double roll, double pitch, double magx, double magy, double magz);
        bool   checkHead(uint8_t check_head[1]);
        bool   checkHeadType(uint8_t head_type[1]);
        bool   checkHeadLength(uint8_t head_type[1], uint8_t check_len[0]);
        bool   checkCRC(uint8_t check_head[1], uint8_t head_type[1], uint8_t check_len[1]);
        void   checkSN(int type);

        // for ros node loop
        Eigen::Matrix3d    _rot_mat;
        Eigen::Quaterniond _rot;
        Eigen::Quaterniond _init_rot;

        bool _first_frame = true;

        bool _if_debug;
        // sum info
        int     _sn_lost   = 0;
        int     _crc_error = 0;
        uint8_t _read_sn   = 0;
        bool    _frist_sn;
        int     _device_type = 1;

        // serial
        serial::Serial _serial; // 声明串口对象
        std::string    _serial_port;
        int            _serial_baud;
        int            _serial_timeout;
        // data
        FDILink::imu_frame_read               _imu_frame;
        FDILink::ahrs_frame_read              _ahrs_frame;
        FDILink::insgps_frame_read            _insgps_frame;
        FDILink::Geodetic_Position_frame_read _geodetic_position_frame;

        // frame name
        std::string _imu_frame_id;

        // topic
        std::string _imu_topic;
        std::string _mag_pose_2d_topic;

        rclcpp::TimerBase::SharedPtr _timer;

        // Publisher
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr       _imu_pub;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr _gps_pub;
        rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr  _mag_pose_pub;

        std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
    }; // ahrsBringup
} // namespace FDILink

#endif
