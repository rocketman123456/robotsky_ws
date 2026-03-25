#include "utils/fps_counter.h"

#include <spdlog/spdlog.h>

#include <numeric>
#include <cmath>

FPSCounter::FPSCounter(bool print_info_)
    : print_info(print_info_)
{
    _freqs.resize(_num_iterations);
    _index = 0;
    _count = 0;
}

void FPSCounter::start()
{
    _prev = Clock::now();
}

void FPSCounter::update()
{
    TimePoint now = Clock::now();
    Duration  dt  = now - _prev;
    _prev         = now;

    double interval = dt.count();
    if (interval > 0.0)
    {
        _freqs[_index % _num_iterations] = 1.0 / interval;
    }

    _index++;
    _count++;

    if (_count % _num_iterations == 0)
    {
        double sum  = std::accumulate(_freqs.begin(), _freqs.end(), 0.0);
        double mean = sum / _num_iterations;

        double sq_sum = 0.0;
        for (double f : _freqs)
        {
            double diff = f - mean;
            sq_sum += diff * diff;
        }
        double variance = sq_sum / _num_iterations;

        if (print_info)
        {
            spdlog::info("Average frequency: {:.1f} Hz, std: {:.2f} Hz", mean, std::sqrt(variance));
        }
    }
}
