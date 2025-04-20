#include "motor/motor_data.h"
#include "robot/robot.h"
#include "utils/utils.h"

#include <rclcpp/rclcpp.hpp>

#include <spdlog/spdlog.h>

#include <chrono>
#include <memory>

// using namespace std::chrono;
using Clock     = std::chrono::high_resolution_clock;
using TimePoint = Clock::time_point;
using Duration  = std::chrono::duration<double>;

std::vector<CanInitInfo> prepare_can()
{
    std::vector<CanInitInfo> can_infos;

    can_infos.push_back({"can0"});
    can_infos.push_back({"can1"});
    can_infos.push_back({"can2"});
    can_infos.push_back({"can3"});

    return can_infos;
}

std::vector<MotorInitInfo> prepare_motor()
{
    std::vector<MotorInitInfo> motor_infos;

    motor_infos.push_back({MotorType::DM, MotorMode::POSITION, 0, 0x01, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::POSITION, 0, 0x02, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::POSITION, 0, 0x05, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::POSITION, 0, 0x08, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});

    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 2, 0x02, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 2, 0x03, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 2, 0x06, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 2, 0x07, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});

    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 3, 0x0a, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 3, 0x0b, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 3, 0x0e, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::RS, MotorMode::POSITION, 3, 0x0f, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});

    motor_infos.push_back({MotorType::DM, MotorMode::VELOCITY, 1, 0x09, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::VELOCITY, 1, 0x0c, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::VELOCITY, 1, 0x0d, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});
    motor_infos.push_back({MotorType::DM, MotorMode::VELOCITY, 1, 0x10, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0});

    return motor_infos;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto robot = std::make_shared<Robot>();

    std::vector<CanInitInfo>   can_infos   = prepare_can();
    std::vector<MotorInitInfo> motor_infos = prepare_motor();

    robot->initCAN(can_infos);
    robot->initMotors(motor_infos);

    double frequency_hz = 500;
    auto   interval     = Duration(1.0 / frequency_hz);
    auto   next_time    = Clock::now() + interval;

    const int num_iterations = 1000;

    std::vector<double> freqs;
    freqs.resize(num_iterations + 1);

    set_thread(0, pthread_self());

    uint64_t count = 0;
    uint64_t index = 0;

    // 记录第一次迭代的起始时间
    TimePoint prev = Clock::now();

    try
    {
        while (rclcpp::ok())
        {
            // TODO : task

            rclcpp::spin_some(robot);

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
                    // freqs.push_back(1.0 / interval);
                    freqs[index] = 1.0 / interval;
                }
            }

            if (count > 0 && count % num_iterations == 0)
            {
                // 计算平均频率
                double sum = std::accumulate(freqs.begin(), freqs.end(), 0.0);
                // double mean = sum / (freqs.size());
                double mean = sum / (index + 1);

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

                // freqs.clear();
                // freqs.reserve(num_iterations + 1);

                index = -1;
            }

            count++;
            index++;

            // 等待直到下一个时间点
            std::this_thread::sleep_until(next_time);
            next_time += interval;
        }
    }
    catch (std::runtime_error& e)
    {
        spdlog::warn("runtime error!");
    }

    rclcpp::shutdown();
}
