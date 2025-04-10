#pragma once

#include <Eigen/Eigen>

#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <stdio.h>
#include <string>
#include <vector>

struct can_init_info_t
{
    can_init_info_t(const std::string& port)
        : can_port(port)
    {}

    std::string can_port;
};

class CanDriver
{
public:
    CanDriver()  = default;
    ~CanDriver() = default;

    void initialize(const std::vector<can_init_info_t>& infos);
    void finalize();

    void send(int can_index, can_frame& frame);
    void receive(int can_index, can_frame& frame);

    int getSocket(int index) { return _sockets[index]; }

private:
    int initCAN(const std::string& port);

private:
    const size_t k_can_size = sizeof(struct can_frame);

    std::vector<int> _sockets;
};
