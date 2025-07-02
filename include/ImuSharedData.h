#pragma once
#include <array>
#include <mutex>

struct ImuSharedData {
    std::array<double, 4> quaternion = {1, 0, 0, 0};
    std::array<double, 3> rpy = {0, 0, 0};
    std::array<double, 3> gyro = {0, 0, 0};
    std::array<double, 3> accel = {0, 0, 0};
    std::mutex mtx;
};