#include "imu/ahrs_driver.h"
#include "imu/crc_table.h"

#include <spdlog/spdlog.h>

namespace FDILink
{
    AhrsBringup::AhrsBringup()
        : Node("imu_node")
        , _frist_sn(false)
        , _serial_timeout(20)
    {
        using namespace std::chrono_literals;

        // topic_name & frame_id
        _if_debug    = false;
        _device_type = 1;

        _imu_topic         = "/robotsky_imu";
        _imu_frame_id      = "imu_link";
        _mag_pose_2d_topic = "/mag_pose_2d";

        // serial
        _serial_port = "/dev/ttyUSB0";
        _serial_baud = 921600;

        // publisher
        _imu_pub      = this->create_publisher<sensor_msgs::msg::Imu>(_imu_topic, 10);
        _gps_pub      = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
        _mag_pose_pub = this->create_publisher<geometry_msgs::msg::Pose2D>(_mag_pose_2d_topic, 10);

        // setp up serial
        try
        {
            serial::Timeout time_out = serial::Timeout::simpleTimeout(_serial_timeout);
            _serial.setPort(_serial_port);
            _serial.setBaudrate(_serial_baud);
            _serial.setFlowcontrol(serial::flowcontrol_none);
            _serial.setParity(serial::parity_none); // default is parity_none
            _serial.setStopbits(serial::stopbits_one);
            _serial.setBytesize(serial::eightbits);
            _serial.setTimeout(time_out);
            _serial.open();
        }
        catch (serial::IOException& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to open port.");
            exit(0);
        }

        if (_serial.isOpen())
        {
            RCLCPP_INFO(this->get_logger(), "Serial Port initialized");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to initial Serial port.");
            exit(0);
        }

        // _timer = this->create_wall_timer(5ms, std::bind(&AhrsBringup::processLoop, this));
        processLoop();
    }

    AhrsBringup::~AhrsBringup()
    {
        if (_serial.isOpen())
            _serial.close();
    }

    bool AhrsBringup::checkHead(uint8_t check_head[1])
    {
        size_t head_s = _serial.read(check_head, 1);
        if (_if_debug)
        {
            if (head_s != 1)
            {
                RCLCPP_ERROR(this->get_logger(), "Read serial port time out! can't read pack head.");
            }
            std::cout << std::endl;
            std::cout << "check_head: " << std::hex << (int)check_head[0] << std::dec << std::endl;
        }
        if (check_head[0] != FRAME_HEAD)
        {
            return false;
        }
        return true;
    }

    bool AhrsBringup::checkHeadType(uint8_t head_type[1])
    {
        size_t type_s = _serial.read(head_type, 1);
        if (_if_debug)
        {
            RCLCPP_ERROR(this->get_logger(), "head_type:  '%x'", (int)head_type[0]);
        }
        if (head_type[0] != TYPE_IMU && head_type[0] != TYPE_AHRS && head_type[0] != TYPE_INSGPS && head_type[0] != TYPE_GEODETIC_POS && head_type[0] != 0x50 &&
            head_type[0] != TYPE_GROUND)
        {
            RCLCPP_WARN(this->get_logger(), "head_type error:  '%x'", (int)head_type[0]);
            return false;
        }
        return true;
    }

    bool AhrsBringup::checkHeadLength(uint8_t head_type[1], uint8_t check_len[0])
    {
        size_t len_s = _serial.read(check_len, 1);
        if (_if_debug)
        {
            RCLCPP_WARN(this->get_logger(), "check_len:  '%d'", (int)check_len[0]);
        }
        if (head_type[0] == TYPE_IMU && check_len[0] != IMU_LEN)
        {
            RCLCPP_WARN(this->get_logger(), "head_len error (imu)");
            return false;
        }
        else if (head_type[0] == TYPE_AHRS && check_len[0] != AHRS_LEN)
        {
            RCLCPP_WARN(this->get_logger(), "head_len error (ahrs)");
            return false;
        }
        else if (head_type[0] == TYPE_INSGPS && check_len[0] != INSGPS_LEN)
        {
            RCLCPP_WARN(this->get_logger(), "head_len error (insgps)");
            return false;
        }
        else if (head_type[0] == TYPE_GEODETIC_POS && check_len[0] != GEODETIC_POS_LEN)
        {
            RCLCPP_WARN(this->get_logger(), "head_len error (GEODETIC_POS)");
            return false;
        }
        else if (head_type[0] == TYPE_GROUND || head_type[0] == 0x50) // 未知数据，防止记录失败
        {
            uint8_t ground_sn[1];
            size_t  ground_sn_s = _serial.read(ground_sn, 1);
            if (++_read_sn != ground_sn[0])
            {
                if (ground_sn[0] < _read_sn)
                {
                    if (_if_debug)
                    {
                        RCLCPP_WARN(this->get_logger(), "detected sn lost.");
                    }
                    _sn_lost += 256 - (int)(_read_sn - ground_sn[0]);
                    _read_sn = ground_sn[0];
                    // continue;
                }
                else
                {
                    if (_if_debug)
                    {
                        RCLCPP_WARN(this->get_logger(), "detected sn lost.");
                    }
                    _sn_lost += (int)(ground_sn[0] - _read_sn);
                    _read_sn = ground_sn[0];
                    // continue;
                }
            }
            uint8_t ground_ignore[500];
            size_t  ground_ignore_s = _serial.read(ground_ignore, (check_len[0] + 4));
            return false;
        }
        return true;
    }

    bool AhrsBringup::checkCRC(uint8_t check_head[1], uint8_t head_type[1], uint8_t check_len[1])
    {
        // read head sn
        uint8_t check_sn[1]     = {0xff};
        size_t  sn_s            = _serial.read(check_sn, 1);
        uint8_t head_crc8[1]    = {0xff};
        size_t  crc8_s          = _serial.read(head_crc8, 1);
        uint8_t head_crc16_H[1] = {0xff};
        uint8_t head_crc16_L[1] = {0xff};
        size_t  crc16_H_s       = _serial.read(head_crc16_H, 1);
        size_t  crc16_L_s       = _serial.read(head_crc16_L, 1);
        if (_if_debug)
        {
            std::cout << "check_sn: " << std::hex << (int)check_sn[0] << std::dec << std::endl;
            std::cout << "head_crc8: " << std::hex << (int)head_crc8[0] << std::dec << std::endl;
            std::cout << "head_crc16_H: " << std::hex << (int)head_crc16_H[0] << std::dec << std::endl;
            std::cout << "head_crc16_L: " << std::hex << (int)head_crc16_L[0] << std::dec << std::endl;
        }
        // put header & check crc8 & count sn lost
        if (head_type[0] == TYPE_IMU)
        {
            _imu_frame.frame.header.header_start   = check_head[0];
            _imu_frame.frame.header.data_type      = head_type[0];
            _imu_frame.frame.header.data_size      = check_len[0];
            _imu_frame.frame.header.serial_num     = check_sn[0];
            _imu_frame.frame.header.header_crc8    = head_crc8[0];
            _imu_frame.frame.header.header_crc16_h = head_crc16_H[0];
            _imu_frame.frame.header.header_crc16_l = head_crc16_L[0];

            uint8_t CRC8 = CRC8_Table(_imu_frame.read_buf.frame_header, 4);
            if (CRC8 != _imu_frame.frame.header.header_crc8)
            {
                RCLCPP_WARN(this->get_logger(), "header_crc8 error");
                return false;
            }
            if (!_frist_sn)
            {
                _read_sn  = _imu_frame.frame.header.serial_num - 1;
                _frist_sn = true;
            }
            // check sn
            AhrsBringup::checkSN(TYPE_IMU);
        }
        else if (head_type[0] == TYPE_AHRS)
        {
            _ahrs_frame.frame.header.header_start   = check_head[0];
            _ahrs_frame.frame.header.data_type      = head_type[0];
            _ahrs_frame.frame.header.data_size      = check_len[0];
            _ahrs_frame.frame.header.serial_num     = check_sn[0];
            _ahrs_frame.frame.header.header_crc8    = head_crc8[0];
            _ahrs_frame.frame.header.header_crc16_h = head_crc16_H[0];
            _ahrs_frame.frame.header.header_crc16_l = head_crc16_L[0];

            uint8_t CRC8 = CRC8_Table(_ahrs_frame.read_buf.frame_header, 4);
            if (CRC8 != _ahrs_frame.frame.header.header_crc8)
            {
                RCLCPP_WARN(this->get_logger(), "header_crc8 error");
                return false;
            }
            if (!_frist_sn)
            {
                _read_sn  = _ahrs_frame.frame.header.serial_num - 1;
                _frist_sn = true;
            }
            // check sn
            AhrsBringup::checkSN(TYPE_AHRS);
        }
        else if (head_type[0] == TYPE_INSGPS)
        {
            _insgps_frame.frame.header.header_start   = check_head[0];
            _insgps_frame.frame.header.data_type      = head_type[0];
            _insgps_frame.frame.header.data_size      = check_len[0];
            _insgps_frame.frame.header.serial_num     = check_sn[0];
            _insgps_frame.frame.header.header_crc8    = head_crc8[0];
            _insgps_frame.frame.header.header_crc16_h = head_crc16_H[0];
            _insgps_frame.frame.header.header_crc16_l = head_crc16_L[0];

            uint8_t CRC8 = CRC8_Table(_insgps_frame.read_buf.frame_header, 4);
            if (CRC8 != _insgps_frame.frame.header.header_crc8)
            {
                RCLCPP_WARN(this->get_logger(), "header_crc8 error");
                return false;
            }
            else if (_if_debug)
            {
                RCLCPP_INFO(this->get_logger(), "header_crc8 matched");
            }

            AhrsBringup::checkSN(TYPE_INSGPS);
        }
        else if (head_type[0] == TYPE_GEODETIC_POS)
        {
            _geodetic_position_frame.frame.header.header_start   = check_head[0];
            _geodetic_position_frame.frame.header.data_type      = head_type[0];
            _geodetic_position_frame.frame.header.data_size      = check_len[0];
            _geodetic_position_frame.frame.header.serial_num     = check_sn[0];
            _geodetic_position_frame.frame.header.header_crc8    = head_crc8[0];
            _geodetic_position_frame.frame.header.header_crc16_h = head_crc16_H[0];
            _geodetic_position_frame.frame.header.header_crc16_l = head_crc16_L[0];

            uint8_t crc8 = CRC8_Table(_geodetic_position_frame.read_buf.frame_header, 4);
            if (crc8 != _geodetic_position_frame.frame.header.header_crc8)
            {
                RCLCPP_INFO(this->get_logger(), "header_crc8 error");
                return false;
            }
            if (!_frist_sn)
            {
                _read_sn  = _geodetic_position_frame.frame.header.serial_num - 1;
                _frist_sn = true;
            }

            AhrsBringup::checkSN(TYPE_GEODETIC_POS);
        }
        // put header & check crc16 & count sn lost
        if (head_type[0] == TYPE_IMU)
        {
            uint16_t head_crc16_l = _imu_frame.frame.header.header_crc16_l;
            uint16_t head_crc16_h = _imu_frame.frame.header.header_crc16_h;
            uint16_t head_crc16   = head_crc16_l + (head_crc16_h << 8);
            size_t   data_s       = _serial.read(_imu_frame.read_buf.read_msg, (IMU_LEN + 1)); // 48+1
            uint16_t crc16        = CRC16_Table(_imu_frame.frame.data.data_buff, IMU_LEN);
            if (_if_debug)
            {
                std::cout << "CRC16:        " << std::hex << (int)crc16 << std::dec << std::endl;
                std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
                std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
                std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
                bool if_right = ((int)head_crc16 == (int)crc16);
                std::cout << "if_right: " << if_right << std::endl;
            }

            if (head_crc16 != crc16)
            {
                RCLCPP_WARN(this->get_logger(), "check crc16 faild(imu).");
                return false;
            }
            else if (_imu_frame.frame.frame_end != FRAME_END)
            {
                RCLCPP_WARN(this->get_logger(), "check frame end.");
                return false;
            }
        }
        else if (head_type[0] == TYPE_AHRS)
        {
            uint16_t head_crc16_l = _ahrs_frame.frame.header.header_crc16_l;
            uint16_t head_crc16_h = _ahrs_frame.frame.header.header_crc16_h;
            uint16_t head_crc16   = head_crc16_l + (head_crc16_h << 8);
            size_t   data_s       = _serial.read(_ahrs_frame.read_buf.read_msg, (AHRS_LEN + 1)); // 48+1
            uint16_t crc16        = CRC16_Table(_ahrs_frame.frame.data.data_buff, AHRS_LEN);
            if (_if_debug)
            {
                std::cout << "CRC16:        " << std::hex << (int)crc16 << std::dec << std::endl;
                std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
                std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
                std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
                bool if_right = ((int)head_crc16 == (int)crc16);
                std::cout << "if_right: " << if_right << std::endl;
            }

            if (head_crc16 != crc16)
            {
                RCLCPP_WARN(this->get_logger(), "check crc16 faild(ahrs).");
                return false;
            }
            else if (_ahrs_frame.frame.frame_end != FRAME_END)
            {
                RCLCPP_WARN(this->get_logger(), "check frame end.");
                return false;
            }
        }
        else if (head_type[0] == TYPE_INSGPS)
        {
            uint16_t head_crc16_l = _insgps_frame.frame.header.header_crc16_l;
            uint16_t head_crc16_h = _insgps_frame.frame.header.header_crc16_h;
            uint16_t head_crc16   = head_crc16_l + (head_crc16_h << 8);
            size_t   data_s       = _serial.read(_insgps_frame.read_buf.read_msg, (INSGPS_LEN + 1)); // 48+1
            uint16_t crc16        = CRC16_Table(_insgps_frame.frame.data.data_buff, INSGPS_LEN);
            if (_if_debug)
            {
                std::cout << "CRC16:        " << std::hex << (int)crc16 << std::dec << std::endl;
                std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
                std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
                std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
                bool if_right = ((int)head_crc16 == (int)crc16);
                std::cout << "if_right: " << if_right << std::endl;
            }

            if (head_crc16 != crc16)
            {
                RCLCPP_WARN(this->get_logger(), "check crc16 faild(ahrs).");
                return false;
            }
            else if (_insgps_frame.frame.frame_end != FRAME_END)
            {
                RCLCPP_WARN(this->get_logger(), "check frame end.");
                return false;
            }
        }
        else if (head_type[0] == TYPE_GEODETIC_POS)
        {
            uint16_t head_crc16_l = _geodetic_position_frame.frame.header.header_crc16_l;
            uint16_t head_crc16_h = _geodetic_position_frame.frame.header.header_crc16_h;
            uint16_t head_crc16   = head_crc16_l + (head_crc16_h << 8);
            size_t   data_s       = _serial.read(_geodetic_position_frame.read_buf.read_msg, (GEODETIC_POS_LEN + 1)); // 24+1

            uint16_t CRC16 = CRC16_Table(_geodetic_position_frame.frame.data.data_buff, GEODETIC_POS_LEN);
            if (_if_debug)
            {
                std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
                std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
                std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
                std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
                bool if_right = ((int)head_crc16 == (int)CRC16);
                std::cout << "if_right: " << if_right << std::endl;
            }

            if (head_crc16 != CRC16)
            {
                RCLCPP_WARN(this->get_logger(), "check crc16 faild(gps).");
                return false;
            }
            else if (_geodetic_position_frame.frame.frame_end != FRAME_END)
            {
                RCLCPP_WARN(this->get_logger(), "check frame end.");
                return false;
            }
        }
        return true;
    }

    void AhrsBringup::checkSN(int type)
    {
        switch (type)
        {
            case TYPE_IMU:
                if (++_read_sn != _imu_frame.frame.header.serial_num)
                {
                    if (_imu_frame.frame.header.serial_num < _read_sn)
                    {
                        _sn_lost += 256 - (int)(_read_sn - _imu_frame.frame.header.serial_num);
                        if (_if_debug)
                        {
                            RCLCPP_WARN(this->get_logger(), "detected sn lost.");
                        }
                    }
                    else
                    {
                        _sn_lost += (int)(_imu_frame.frame.header.serial_num - _read_sn);
                        if (_if_debug)
                        {
                            RCLCPP_WARN(this->get_logger(), "detected sn lost.");
                        }
                    }
                }
                _read_sn = _imu_frame.frame.header.serial_num;
                break;

            case TYPE_AHRS:
                if (++_read_sn != _ahrs_frame.frame.header.serial_num)
                {
                    if (_ahrs_frame.frame.header.serial_num < _read_sn)
                    {
                        _sn_lost += 256 - (int)(_read_sn - _ahrs_frame.frame.header.serial_num);
                        if (_if_debug)
                        {
                            RCLCPP_WARN(this->get_logger(), "detected sn lost.");
                        }
                    }
                    else
                    {
                        _sn_lost += (int)(_ahrs_frame.frame.header.serial_num - _read_sn);
                        if (_if_debug)
                        {
                            RCLCPP_WARN(this->get_logger(), "detected sn lost.");
                        }
                    }
                }
                _read_sn = _ahrs_frame.frame.header.serial_num;
                break;

            case TYPE_INSGPS:
                if (++_read_sn != _insgps_frame.frame.header.serial_num)
                {
                    if (_insgps_frame.frame.header.serial_num < _read_sn)
                    {
                        _sn_lost += 256 - (int)(_read_sn - _insgps_frame.frame.header.serial_num);
                        if (_if_debug)
                        {
                            RCLCPP_WARN(this->get_logger(), "detected sn lost.");
                        }
                    }
                    else
                    {
                        _sn_lost += (int)(_insgps_frame.frame.header.serial_num - _read_sn);
                        if (_if_debug)
                        {
                            RCLCPP_WARN(this->get_logger(), "detected sn lost.");
                        }
                    }
                }
                _read_sn = _insgps_frame.frame.header.serial_num;
                break;

            case TYPE_GEODETIC_POS:
                if (++_read_sn != _geodetic_position_frame.frame.header.serial_num)
                {
                    if (_geodetic_position_frame.frame.header.serial_num < _read_sn)
                    {
                        _sn_lost += 256 - (_read_sn - _geodetic_position_frame.frame.header.serial_num);
                        if (_if_debug)
                        {
                            RCLCPP_WARN(this->get_logger(), "detected sn lost.");
                        }
                    }
                    else
                    {
                        _sn_lost += (_geodetic_position_frame.frame.header.serial_num - _read_sn);
                        if (_if_debug)
                        {
                            RCLCPP_WARN(this->get_logger(), "detected sn lost.");
                        }
                    }
                }
                _read_sn = _geodetic_position_frame.frame.header.serial_num;
                break;

            default:
                break;
        }
    }

    double AhrsBringup::magCalculateYaw(double roll, double pitch, double magx, double magy, double magz)
    {
        double magyaw = 0;
        double temp1  = magy * cos(roll) + magz * sin(roll);
        double temp2  = magx * cos(pitch) + magy * sin(pitch) * sin(roll) - magz * sin(pitch) * cos(roll);
        magyaw        = atan2(-temp1, temp2);
        if (magyaw < 0)
        {
            magyaw = magyaw + 2 * PI;
        }
        return magyaw;
    }

    void AhrsBringup::initialize()
    {
        RCLCPP_INFO(this->get_logger(), "AhrsBringup::processLoop: start");

        // convert imu frame into body frame
        // _rot_mat << 0, -1, 0, 1, 0, 0, 0, 0, 1;
        // _rot_mat << 1, 0, 0, 0, 1, 0, 0, 0, 1;
        // _rot_mat << 1, 0, 0, 0, -1, 0, 0, 0, -1;
        _rot_mat << 1, 0, 0, 0, 1, 0, 0, 0, 1;
        _rot = Eigen::Quaterniond(_rot_mat);
        // std::cout << "rot : " << _rot.toRotationMatrix() << std::endl;
    }

    void AhrsBringup::processLoop()
    {
        if (!_serial.isOpen())
        {
            RCLCPP_WARN(this->get_logger(), "serial unopen");
            exit(0);
        }
        // check head start
        uint8_t check_head[1] = {0xff};

        bool result = checkHead(check_head);
        if (!result)
        {
            return;
        }

        // check head type
        uint8_t head_type[1] = {0xff};
        result               = checkHeadType(head_type);
        if (!result)
        {
            return;
        }

        // check head length
        uint8_t check_len[1] = {0xff};
        result               = checkHeadLength(head_type, check_len);
        if (!result)
        {
            return;
        }

        result = checkCRC(check_head, head_type, check_len);
        if (!result)
        {
            return;
        }

        // publish magyaw topic
        if (head_type[0] == TYPE_AHRS)
        {
            // publish imu topic
            sensor_msgs::msg::Imu imu_data;
            imu_data.header.stamp    = this->now();
            imu_data.header.frame_id = _imu_frame_id;
            Eigen::Quaterniond q_ahrs(
                _ahrs_frame.frame.data.data_pack.Qw,
                _ahrs_frame.frame.data.data_pack.Qx,
                _ahrs_frame.frame.data.data_pack.Qy,
                _ahrs_frame.frame.data.data_pack.Qz
            );
            Eigen::Quaterniond q_r = Eigen::AngleAxisd(3.14159, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(3.14159, Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(0.00000, Eigen::Vector3d::UnitX());
            Eigen::Quaterniond q_rr = Eigen::AngleAxisd(0.00000, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0.00000, Eigen::Vector3d::UnitY()) *
                                      Eigen::AngleAxisd(3.14159, Eigen::Vector3d::UnitX());
            Eigen::Quaterniond q_min_rr = Eigen::AngleAxisd(3.14159 / 2, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0.00000, Eigen::Vector3d::UnitY()) *
                                          Eigen::AngleAxisd(3.14159, Eigen::Vector3d::UnitX());
            if (_device_type == 0) // 未经变换的原始数据
            {
                // std::cout << "device type: raw" << std::endl;
                imu_data.orientation.w         = _ahrs_frame.frame.data.data_pack.Qw;
                imu_data.orientation.x         = _ahrs_frame.frame.data.data_pack.Qx;
                imu_data.orientation.y         = _ahrs_frame.frame.data.data_pack.Qy;
                imu_data.orientation.z         = _ahrs_frame.frame.data.data_pack.Qz;
                imu_data.angular_velocity.x    = _ahrs_frame.frame.data.data_pack.RollSpeed;
                imu_data.angular_velocity.y    = _ahrs_frame.frame.data.data_pack.PitchSpeed;
                imu_data.angular_velocity.z    = _ahrs_frame.frame.data.data_pack.HeadingSpeed;
                imu_data.linear_acceleration.x = _imu_frame.frame.data.data_pack.accelerometer_x;
                imu_data.linear_acceleration.y = _imu_frame.frame.data.data_pack.accelerometer_y;
                imu_data.linear_acceleration.z = _imu_frame.frame.data.data_pack.accelerometer_z;
                // cout << "q1: " << imu_data.orientation.w << ", q2: " << imu_data.orientation.x << ", q3: " <<
                // imu_data.orientation.y << ", q4: " << imu_data.orientation.z << endl;
            }
            else if (_device_type == 1) // imu单品ROS标准下的坐标变换
            {
                // std::cout << "device type: imu" << std::endl;
                Eigen::Quaterniond q_out       = q_r * q_ahrs * q_rr;
                imu_data.orientation.w         = q_out.w();
                imu_data.orientation.x         = q_out.x();
                imu_data.orientation.y         = q_out.y();
                imu_data.orientation.z         = q_out.z();
                imu_data.angular_velocity.x    = _ahrs_frame.frame.data.data_pack.RollSpeed;
                imu_data.angular_velocity.y    = -_ahrs_frame.frame.data.data_pack.PitchSpeed;
                imu_data.angular_velocity.z    = -_ahrs_frame.frame.data.data_pack.HeadingSpeed;
                imu_data.linear_acceleration.x = _imu_frame.frame.data.data_pack.accelerometer_x;
                imu_data.linear_acceleration.y = -_imu_frame.frame.data.data_pack.accelerometer_y;
                imu_data.linear_acceleration.z = -_imu_frame.frame.data.data_pack.accelerometer_z;
            }

            if (_first_frame)
            {
                _first_frame = false;
                _init_rot    = Eigen::Quaterniond(imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z);
                _init_rot    = _init_rot * _rot;

                Eigen::Matrix3d   mat = _init_rot.normalized().toRotationMatrix();
                Eigen::AngleAxisd new_angle_axis(mat);
                _init_rot = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(new_angle_axis.angle() * new_angle_axis.axis()(2), Eigen::Vector3d::UnitZ());
                // std::cout << new_angle_axis.angle() << ", " << new_angle_axis.axis() << "\n\n";
                // spdlog::info("init rot: {}, {}, {}, {}", new_angle_axis.angle(), new_angle_axis.axis()(0), new_angle_axis.axis()(1), new_angle_axis.axis()(2));
                RCLCPP_INFO(this->get_logger(), "init rot: %f, %f, %f, %f", new_angle_axis.angle(), new_angle_axis.axis()(0), new_angle_axis.axis()(1), new_angle_axis.axis()(2));
            }

            geometry_msgs::msg::Pose  pose;
            geometry_msgs::msg::Twist twist;

            // convert imu frame into body frame
            Eigen::Quaterniond quat(imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z);
            Eigen::Quaterniond delta_rot;
            delta_rot = _rot * _init_rot.conjugate();
            // quat      = quat * delta_rot;
            quat = quat * _rot; // * rot.conjugate();
            quat = _init_rot.conjugate() * quat;

            imu_data.orientation.w = quat.w();
            imu_data.orientation.x = quat.x();
            imu_data.orientation.y = quat.y();
            imu_data.orientation.z = quat.z();

            // RCLCPP_INFO(this->get_logger(), "orientation: %f, %f, %f, %f", quat.w(), quat.x(), quat.y(), quat.z());

            _imu_pub->publish(imu_data);

            pose.orientation.w = quat.w();
            pose.orientation.x = quat.x();
            pose.orientation.y = quat.y();
            pose.orientation.z = quat.z();
            pose.position.x    = 0;
            pose.position.y    = 0;
            pose.position.z    = 0;

            // RCLCPP_INFO(this->get_logger(), "pose: %f, %f, %f, %f", quat.w(), quat.x(), quat.y(), quat.z());

            // transform angular_velocity into body frame
            Eigen::Vector3d w;
            w << imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z;
            w = delta_rot.toRotationMatrix().transpose() * w;

            twist.angular.x = w[0];
            twist.angular.y = w[1];
            twist.angular.z = w[2];
            twist.linear.x  = 0;
            twist.linear.y  = 0;
            twist.linear.z  = 0;

            Eigen::Quaterniond         rpy_q(imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z);
            geometry_msgs::msg::Pose2D pose_2d;
            double                     magx, magy, magz, roll, pitch;
            if (_device_type == 0)
            { // 未经变换的原始数据//
                magx  = _imu_frame.frame.data.data_pack.magnetometer_x;
                magy  = _imu_frame.frame.data.data_pack.magnetometer_y;
                magz  = _imu_frame.frame.data.data_pack.magnetometer_z;
                roll  = _ahrs_frame.frame.data.data_pack.Roll;
                pitch = _ahrs_frame.frame.data.data_pack.Pitch;
            }
            else if (_device_type == 1)
            { // 小车以及imu单品ROS标准下的坐标变换//
                magx = _imu_frame.frame.data.data_pack.magnetometer_x;
                magy = _imu_frame.frame.data.data_pack.magnetometer_y;
                magz = _imu_frame.frame.data.data_pack.magnetometer_z;

                Eigen::Vector3d euler_angle = rpy_q.matrix().eulerAngles(2, 1, 0);
                roll                        = euler_angle[2];
                pitch                       = euler_angle[1];
            }
            double magyaw = magCalculateYaw(0, 0, magx, magy, magz);
            pose_2d.theta = magyaw;
            _mag_pose_pub->publish(pose_2d);
        }
    }
} // namespace FDILink
