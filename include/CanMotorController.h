#pragma once

#include <vector>
#include <atomic>
#include <thread>
#include <initializer_list>
#include "DjiMotor.h"

class CanMotorController {
public:
    // Construct with device and motor IDs
    CanMotorController(const char* device_path, std::initializer_list<int> motor_ids);
    ~CanMotorController();

    void runLoop(std::atomic<bool>& running);
    void setCommand(int id, double dq, double tau, double kd);

    // Optionally expose motors for reading data
    const std::vector<DjiMotor>& getMotors() const { return motors_; }

private:
    bool sendMotorCurrents();
    static void int_to_high_low_bytes(int16_t value, uint8_t& high, uint8_t& low);
    int32_t dev_;
    const char* device_path_;
    std::vector<DjiMotor> motors_;
};