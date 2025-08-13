#pragma once

#include <vector>
#include <atomic>
#include <thread>
#include <initializer_list>
#include <cstdint>
#include <memory>

#include "DjiMotor.h"
#include "u2can/SerialPortCAN.h"

class DMCanMotorController {
public:
    // Construct with device and motor IDs (device_path = serial device, e.g., "/dev/ttyACM0")
    DMCanMotorController(const char* device_path, std::initializer_list<int> motor_ids);
    ~DMCanMotorController();

    void runLoop(std::atomic<bool>& running);
    void setCommand(int id, double dq, double tau, double kd);

    // Optionally expose motors for reading data
    const std::vector<DjiMotor>& getMotors() const { return motors_; }

private:
    bool sendMotorCurrents();
    static void int_to_high_low_bytes(int16_t value, uint8_t& high, uint8_t& low);

    // --- Serial-only helpers (replace usb2can) ---
    bool send_can_std_8(uint32_t can_id, const uint8_t data[8]);    // send one 8-byte standard CAN frame via SerialPort
    bool recv_can_frame(uint32_t& out_can_id, uint8_t out_data[8]);  // try receive one CAN frame (parsed from serial stream)

private:
    int32_t dev_;                        // simple opened-flag (>=0 opened)
    const char* device_path_;
    std::vector<DjiMotor> motors_;

    // Serial transport
    std::shared_ptr<SerialPortCAN> serial_;
};
