#include "utils/fps_counter.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

FPSCounter::FPSCounter(bool print_info_, std::string label, double report_interval_sec)
    : print_info(print_info_)
    , _label(std::move(label))
    , _report_interval_sec(report_interval_sec)
{
}

void FPSCounter::start()
{
    reset_window(Clock::now());
    _has_prev = false;
}

void FPSCounter::reset_window(const TimePoint& now)
{
    _window_start = now;
    _sample_count = 0;
    _sum_freq     = 0.0;
    _sum_sq_freq  = 0.0;
}

void FPSCounter::update()
{
    const TimePoint now = Clock::now();
    if (!_has_prev)
    {
        _prev     = now;
        _has_prev = true;
        return;
    }

    const Duration dt       = now - _prev;
    _prev                   = now;

    const double interval = dt.count();
    if (interval > 0.0)
    {
        const double freq = 1.0 / interval;
        _sum_freq += freq;
        _sum_sq_freq += freq * freq;
        ++_sample_count;
    }

    const double elapsed = std::chrono::duration<double>(now - _window_start).count();
    if (elapsed >= _report_interval_sec && _sample_count > 0)
    {
        const double mean     = _sum_freq / static_cast<double>(_sample_count);
        const double variance = std::max(0.0, _sum_sq_freq / static_cast<double>(_sample_count) - mean * mean);

        if (print_info)
        {
            spdlog::info("{} frequency: mean {:.1f} Hz, std {:.2f} Hz ({} samples)", _label, mean, std::sqrt(variance), _sample_count);
        }

        reset_window(now);
    }
}
