#pragma once

#include <chrono>
#include <vector>

class FPSCounter
{
public:
    FPSCounter(bool print_info = false);
    ~FPSCounter() = default;

    void start();
    void update();

private:
    using Clock     = std::chrono::high_resolution_clock;
    using TimePoint = Clock::time_point;
    using Duration  = std::chrono::duration<double>;

    const int _num_iterations = 1000;

    std::vector<double> _freqs = {};

    uint64_t _index = 0;
    uint64_t _count = 0;

    TimePoint _prev;

    bool _print_info = false;
};
