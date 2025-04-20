#include "utils/fps_counter.h"

#include <spdlog/spdlog.h>

#include <numeric>

FPSCounter::FPSCounter(bool print_info)
{
    _freqs.resize(_num_iterations + 1);
    _index      = 0;
    _print_info = print_info;
}

void FPSCounter::start()
{
    // 记录第一次迭代的起始时间
    _prev = Clock::now();
}

void FPSCounter::update()
{
    // 迭代完成后，记录当前时间
    TimePoint now = Clock::now();
    // 计算两次迭代间隔（秒）
    Duration dt = now - _prev;
    _prev       = now;

    // 跳过第一次，因为还没频率可算
    if (_count > 0)
    {
        double interval = dt.count();
        if (interval > 0.0)
        {
            _freqs[_index] = 1.0 / interval;
        }
    }

    if (_count > 0 && _count % _num_iterations == 0)
    {
        // 计算平均频率
        double sum  = std::accumulate(_freqs.begin(), _freqs.end(), 0.0);
        double mean = sum / (_index + 1);

        // 计算方差
        double sq_sum = 0.0;
        for (double f : _freqs)
        {
            double diff = f - mean;
            sq_sum += diff * diff;
        }
        double variance = sq_sum / (_index + 1);

        if (_print_info)
        {
            spdlog::info("Average frequency: {} Hz", mean);
            spdlog::info("Variance: {} (Hz^2)", variance);
        }

        _index = -1;
    }

    _index++;
    _count++;
}
