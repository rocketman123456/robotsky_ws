#include <rclcpp/rclcpp.hpp>

#include "motor/motor_data.h"
#include "robot/robot.h"

#include <spdlog/spdlog.h>

#include <chrono>
#include <memory>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto robot = std::make_shared<Robot>();

    std::vector<CanInitInfo> can_infos;

    can_infos.push_back({"can0"});
    can_infos.push_back({"can1"});
    can_infos.push_back({"can2"});
    can_infos.push_back({"can3"});

    robot->initCAN(can_infos);

    std::vector<MotorInitInfo> motor_infos;

    motor_infos.push_back({MotorType::DM, MotorMode::POSITION, 0, 0x01, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::POSITION, 0, 0x02, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::POSITION, 0, 0x05, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::POSITION, 0, 0x08, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});

    motor_infos.push_back({MotorType::DM, MotorMode::POSITION, 2, 0x02, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::POSITION, 2, 0x03, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::POSITION, 2, 0x06, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::POSITION, 2, 0x07, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});

    motor_infos.push_back({MotorType::DM, MotorMode::POSITION, 1, 0x09, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::POSITION, 1, 0x0c, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::POSITION, 1, 0x0d, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::POSITION, 1, 0x10, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});

    motor_infos.push_back({MotorType::DM, MotorMode::POSITION, 3, 0x0a, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::POSITION, 3, 0x0b, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::POSITION, 3, 0x0e, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::POSITION, 3, 0x0f, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});

    robot->initMotors(motor_infos);

    // std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("motor_server");

    // rclcpp::spin(robot);

    using namespace std::chrono;
    double frequency_hz = 500;
    auto   interval     = duration<double>(1.0 / frequency_hz);
    auto   next_time    = steady_clock::now() + interval;

    using Clock     = std::chrono::high_resolution_clock;
    using TimePoint = Clock::time_point;
    using Duration  = std::chrono::duration<double>;

    const int num_iterations = 1000;

    std::vector<double> freqs;
    freqs.reserve(num_iterations + 1);

    uint64_t count = 0;

    // 记录第一次迭代的起始时间
    TimePoint prev = Clock::now();

    try
    {
        rclcpp::Rate loop_rate(500);

        while (rclcpp::ok())
        {
            // 迭代完成后，记录当前时间
            TimePoint now = Clock::now();
            // 计算两次迭代间隔（秒）
            Duration dt = now - prev;
            prev        = now;

            // 跳过第一次，因为还没频率可算
            if (count > 0)
            {
                double interval = dt.count();
                if (interval > 0.0)
                {
                    freqs.push_back(1.0 / interval);
                }
            }

            if (count > 0 && count % num_iterations == 0)
            {
                // 计算平均频率
                double sum  = std::accumulate(freqs.begin(), freqs.end(), 0.0);
                double mean = sum / freqs.size();

                // 计算方差
                double sq_sum = 0.0;
                for (double f : freqs)
                {
                    double diff = f - mean;
                    sq_sum += diff * diff;
                }
                double variance = sq_sum / freqs.size();

                spdlog::info("Average frequency: {} Hz", mean);
                spdlog::info("Variance: {} (Hz^2)", variance);

                freqs.clear();
                freqs.reserve(num_iterations + 1);
            }

            count++;

            rclcpp::spin_some(robot);

            // 等待直到下一个时间点
            std::this_thread::sleep_until(next_time);
            next_time += interval;

            // loop_rate.sleep();
        }
    }
    catch (std::runtime_error& e)
    {
        spdlog::warn("runtime error!");
    }

    rclcpp::shutdown();
}
