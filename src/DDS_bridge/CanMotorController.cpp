#include "CanMotorController.h"
#include "usb2can/usb_can.h"
#include <algorithm>
#include <cstdio>
#include <cmath>
#include <chrono>
#include <thread>

CanMotorController::CanMotorController(const char* device_path, std::initializer_list<int> motor_ids)
    : device_path_(device_path), dev_(-1)
{
    for (int id : motor_ids) {
        motors_.emplace_back(id);
    }
    dev_ = openUSBCAN(device_path_);
    if (dev_ < 0) {
        printf("Failed to open USB2CAN device\n");
    }
}

CanMotorController::~CanMotorController() {
    if (dev_ >= 0) {
        closeUSBCAN(dev_);
    }
}

void CanMotorController::setCommand(int id, double dq, double tau, double kd) {
    for (auto& motor : motors_) {
        if (motor.id == id) {
            motor.cmd.dq = dq;
            motor.cmd.tau = tau;
            motor.cmd.kd = kd;
            break;
        }
    }
}

bool CanMotorController::sendMotorCurrents() {
    if (motors_.size() < 2 || dev_ < 0) return false;
    constexpr float torque_const_M3508 = 0.247f;

    float motor1_tau = static_cast<float>(
        motors_[0].cmd.kd * (motors_[0].cmd.dq - motors_[0].data.dq) +
        motors_[0].cmd.tau
    );
    float motor2_tau = static_cast<float>(
        motors_[1].cmd.kd * (motors_[1].cmd.dq - motors_[1].data.dq) +
        motors_[1].cmd.tau
    );

    motor1_tau = std::clamp(motor1_tau, -10.0f, 10.0f);
    motor2_tau = std::clamp(motor2_tau, -10.0f, 10.0f);

    float motor1_current = std::clamp(motor1_tau / torque_const_M3508, -10.0f, 10.0f);
    float motor2_current = std::clamp(motor2_tau / torque_const_M3508, -10.0f, 10.0f);

    int16_t motor1_cmd = static_cast<int16_t>(motor1_current * 16384 / 20);
    int16_t motor2_cmd = static_cast<int16_t>(motor2_current * 16384 / 20);

    uint8_t m1h, m1l, m2h, m2l;
    int_to_high_low_bytes(motor1_cmd, m1h, m1l);
    int_to_high_low_bytes(motor2_cmd, m2h, m2l);

    uint8_t txData[8] = {m1h, m1l, m2h, m2l, 0x00, 0x00, 0x00, 0x00};

    FrameInfo txInfo;
    txInfo.canID = 0x200;
    txInfo.frameType = STANDARD;
    txInfo.dataLength = 8;

    FrameInfo rxInfo;
    uint8_t rxData[8];

    int32_t send_ret = sendUSBCAN(dev_, 2, &txInfo, txData);
    if (send_ret != 17) {
        printf("Failed to send CAN frame\n");
        return false;
    }

    uint8_t channel;
    int32_t recv_ret = readUSBCAN(dev_, &channel, &rxInfo, rxData, 1e6);
    if (recv_ret != 0) {
        printf("Failed to receive CAN frame\n");
        return false;
    }

    // Process received data and write into motor.data
    for (auto& motor : motors_) {
        if ((motor.id + 0x200) == static_cast<int>(rxInfo.canID)) {
            uint16_t raw_angle = (rxData[0] << 8) | rxData[1];
            motor.data.q = static_cast<double>(raw_angle) / 8192.0 * 2.0 * M_PI;

            int16_t raw_vel = (int16_t)((rxData[2] << 8) | rxData[3]);
            if (raw_vel & (1 << 15)) raw_vel -= (1 << 16);
            motor.data.dq = static_cast<double>(raw_vel) / motor.gear_ratio * 2.0 * M_PI / 60.0;

            int16_t raw_current = (int16_t)((rxData[4] << 8) | rxData[5]);
            if (raw_current & (1 << 15)) raw_current -= (1 << 16);
            motor.data.tau = static_cast<double>(raw_current) * torque_const_M3508 / 1000;

            motor.data.temp = static_cast<double>(rxData[6]);
        }
    }

    return true;
}

void CanMotorController::int_to_high_low_bytes(int16_t value, uint8_t& high, uint8_t& low) {
    high = (value >> 8) & 0xFF;
    low = value & 0xFF;
}

void CanMotorController::runLoop(std::atomic<bool>& running) {
    using clock = std::chrono::steady_clock;
    int loop_count = 0;
    auto last_time = clock::now();

    while (running) {
        sendMotorCurrents();
        // ++loop_count;

        // auto now = clock::now();
        // auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_time).count();
        // if (elapsed >= 1) {
        //     printf("CAN loop frequency: %d Hz\n", loop_count);
        //     loop_count = 0;
        //     last_time = now;
        // }
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}