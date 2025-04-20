#include "utils/utils.h"

#include <spdlog/spdlog.h>

#include <pthread.h>
#include <sched.h>

void set_thread(int32_t cpu_core, uint64_t thread_id)
{
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_core, &cpuset); // Set CPU core 0
    if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) == -1)
    {
        spdlog::error("Failed to set CPU affinity");
        exit(-1);
    }

    sched_param param;
    int         policy;
    pthread_getschedparam(thread_id, &policy, &param);
    param.sched_priority = 1; // 20
    pthread_setschedparam(thread_id, SCHED_FIFO, &param);
}