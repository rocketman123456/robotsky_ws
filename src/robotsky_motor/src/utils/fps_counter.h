#pragma once

#include <chrono>
#include <vector>

class FPSCounter
{
public:
    explicit FPSCounter(bool print_info = false);
    ~FPSCounter() = default;

    void start();
    void update();

    bool print_info = false;

private:
    using Clock     = std::chrono::high_resolution_clock;
    using TimePoint = Clock::time_point;
    using Duration  = std::chrono::duration<double>;

    static constexpr int kNumIterations = 1000;
    const int _num_iterations           = kNumIterations;

    std::vector<double> _freqs;

    uint64_t _index = 0;
    uint64_t _count = 0;

    TimePoint _prev;
};
