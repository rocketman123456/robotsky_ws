#pragma once

#include <cstddef>
#include <chrono>
#include <string>

class FPSCounter
{
public:
    explicit FPSCounter(bool print_info = false, std::string label = "loop", double report_interval_sec = 1.0);
    ~FPSCounter() = default;

    void start();
    void update();

    bool print_info = false;

private:
    using Clock     = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;
    using Duration  = std::chrono::duration<double>;

    void reset_window(const TimePoint& now);

    std::string _label;
    double      _report_interval_sec = 1.0;
    std::size_t _sample_count        = 0;
    double      _sum_freq            = 0.0;
    double      _sum_sq_freq         = 0.0;
    bool        _has_prev            = false;
    TimePoint   _prev{};
    TimePoint   _window_start{};
};
