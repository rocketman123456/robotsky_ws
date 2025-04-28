#!/bin/bash
sudo ip link set enp1s0 up
sudo ip addr add 192.168.8.100/24 dev enp1s0
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
sudo ip link set can2 up type can bitrate 1000000
sudo ip link set can3 up type can bitrate 1000000
sudo chmod 666 /dev/ttyUSB0
