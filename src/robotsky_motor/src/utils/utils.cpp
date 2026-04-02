#include "utils/utils.h"

#include <spdlog/spdlog.h>

#include <pthread.h>
#include <sched.h>

void set_thread(int32_t cpu_core, uint64_t thread_id)
{
    (void)thread_id;

    if (cpu_core < 0)
    {
        return;
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_core, &cpuset);
    if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) == -1)
    {
        spdlog::warn("Failed to set CPU affinity to core {}", cpu_core);
        return;
    }

    sched_param param {};
    param.sched_priority = 1;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0)
    {
        spdlog::warn("Failed to enable SCHED_FIFO for current thread");
    }
}
