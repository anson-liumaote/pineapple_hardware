#pragma once
#include <array>
#include <condition_variable>
#include <mutex>

struct ImuSharedData {
    int id = 0;
    std::array<double, 4> quaternion = {1, 0, 0, 0};
    std::array<double, 3> rpy = {0, 0, 0};
    std::array<double, 3> gyro = {0, 0, 0};
    std::array<double, 3> accel = {0, 0, 0};
    std::mutex mtx;
};