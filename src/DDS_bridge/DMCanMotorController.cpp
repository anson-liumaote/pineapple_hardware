// DJI motor control using DM usb2can module

#include "DMCanMotorController.h"
#include "u2can/SerialPortCAN.h" 
#include <algorithm>
#include <cstdio>
#include <cmath>
#include <chrono>
#include <thread>
#include <array>
#include <cstring>

// define data struct
#pragma pack(push, 1)
struct CAN_Receive_Frame {
    uint8_t  FrameHeader;         // 0xAA
    uint8_t  CMD;                 // 0x11: receive success, 0x01/0x02/0x03/0xEE 其他狀態
    uint8_t  canDataLen : 6;      // 資料長度
    uint8_t  canIde     : 1;      // 0: 標準帧 1: 擴展帧
    uint8_t  canRtr     : 1;      // 0: 資料  1: 遠端
    uint32_t canId;               // 回傳的 CAN ID
    uint8_t  canData[8];          // 回傳資料
    uint8_t  frameEnd;            // 0x55
};

struct CAN_Send_Frame {
    uint8_t  FrameHeader[2] = {0x55, 0xAA}; // 帧頭
    uint8_t  FrameLen       = 0x1e;         // 固定 0x1e (對應 damiao.h)
    uint8_t  CMD            = 0x03;         // 3: 非回覆式 CAN 轉發（不回送發送狀態）
    uint32_t sendTimes      = 1;            // 發送次數
    uint32_t timeInterval   = 1;           // 間隔 (ms)
    uint8_t  IDType         = 0;            // 0: 標準帧 1: 擴展帧
    uint32_t canId          = 0x01;         // 寫入實際 CAN ID
    uint8_t  frameType      = 0;            // 0: 資料帧 1: 遠端帧
    uint8_t  len            = 0x08  ;         // 資料長度
    uint8_t  idAcc          = 0;
    uint8_t  dataAcc        = 0;
    uint8_t  data[8]        = {0};
    uint8_t  crc            = 0;            // 未校驗，保留

    void modify(uint32_t id, const uint8_t* bytes8) {
        canId = id;
        std::memcpy(data, bytes8, 8);
    }
};
#pragma pack(pop)


DMCanMotorController::DMCanMotorController(const char* device_path, std::initializer_list<int> motor_ids)
    : device_path_(device_path), dev_(-1)
{
    for (int id : motor_ids) {
        motors_.emplace_back(id);
    }

    // initial serial port
    try {
        serial_ = std::make_shared<SerialPortCAN>(device_path_, B921600);
        dev_ = 1;
    } catch (...) {
        printf("Failed to open serial port: %s\n", device_path_);
        dev_ = -1;
    }

    // create recurrent thread
    CanMotorControllerThreadPtr = CreateRecurrentThreadEx("can_motors", UT_CPU_ID_NONE, 200, &DMCanMotorController::sendMotorCurrents, this);
    std::cout << "[DDS_Bridge] DDS publisher and subscriber initialized." << std::endl;
}

DMCanMotorController::~DMCanMotorController() {
    
}

void DMCanMotorController::setCommand(int id, double dq, double tau, double kd) {
    for (auto& motor : motors_) {
        if (motor.id == id) {
            motor.cmd.dq  = dq;
            motor.cmd.tau = tau;
            motor.cmd.kd  = kd;
            break;
        }
    }
}


void DMCanMotorController::int_to_high_low_bytes(int16_t value, uint8_t& high, uint8_t& low) {
    high = static_cast<uint8_t>((value >> 8) & 0xFF);
    low  = static_cast<uint8_t>( value       & 0xFF);
}

bool DMCanMotorController::send_can_std_8(uint32_t can_id, const uint8_t data[8]) {
    if (!serial_) return false;
    CAN_Send_Frame frame;
    frame.modify(can_id, data);
    ssize_t n = serial_->send(reinterpret_cast<const uint8_t*>(&frame), sizeof(frame));
    return (n == static_cast<ssize_t>(sizeof(frame)));
}

bool DMCanMotorController::recv_can_frame(uint32_t& out_can_id, uint8_t out_data[8]) {
    if (!serial_) return false;

    CAN_Receive_Frame rx{};
    
    serial_->recv(reinterpret_cast<uint8_t*>(&rx), 0xAA, sizeof(CAN_Receive_Frame));

    // check received data
    if (rx.FrameHeader != 0xAA)           return false;
    if (rx.frameEnd   != 0x55)            return false;
    if (rx.CMD        != 0x11)            return false;  // 0x11: receive success
    if (rx.canDataLen  > 8)               return false;  // 我們只收 0~8
    
    out_can_id = rx.canId;
    std::memcpy(out_data, rx.canData, 8);
    return true;
}
int count = 0;
bool DMCanMotorController::sendMotorCurrents() {
    if (motors_.size() < 2 || dev_ < 0) return false;
    constexpr float torque_const_M3508 = 0.246f;

    // auto now = std::chrono::steady_clock::now();
    // auto us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
    // std::cout << "t_loop: " << us << std::endl;

    // velocity controller
    float motor1_tau = static_cast<float>(
        motors_[0].cmd.kd * (motors_[0].cmd.dq - motors_[0].data.dq) +
        motors_[0].cmd.tau
    );
    float motor2_tau = static_cast<float>(
        motors_[1].cmd.kd * (motors_[1].cmd.dq - motors_[1].data.dq) +
        motors_[1].cmd.tau
    );

    // calibrate motor torque
    motor1_tau *= 1.2f;
    motor2_tau *= 1.2f;
    motor1_tau = std::clamp(motor1_tau, -10.0f, 10.0f);
    motor2_tau = std::clamp(motor2_tau, -10.0f, 10.0f);

    // convert torque to current
    float motor1_current = std::clamp(motor1_tau / torque_const_M3508, -20.0f, 20.0f);
    float motor2_current = std::clamp(motor2_tau / torque_const_M3508, -20.0f, 20.0f);

    int16_t motor1_cmd = static_cast<int16_t>(motor1_current * 16384 / 20);
    int16_t motor2_cmd = static_cast<int16_t>(motor2_current * 16384 / 20);

    uint8_t m1h, m1l, m2h, m2l;
    int_to_high_low_bytes(motor1_cmd, m1h, m1l);
    int_to_high_low_bytes(motor2_cmd, m2h, m2l);

    // send data frame
    uint8_t txData[8] = {m1h, m1l, m2h, m2l, 0x00, 0x00, 0x00, 0x00};

    // thread running in 5k Hz, send can command in 1k Hz
    if (count==5) {  
        // now = std::chrono::steady_clock::now();
        // us = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        // std::cout << "t_send: " << us << std::endl;
        if (!send_can_std_8(/*can_id*/0x200, txData)) {
            printf("Failed to send CAN frame (serial)\n");
            return false;
        }
        count = 0;
    }
    else count++;

    uint32_t rx_can_id = 0;
    uint8_t  rxData[8] = {0};

    if (!recv_can_frame(rx_can_id, rxData)) {
        return true; 
    }

    // extract data from can frame
    for (auto& motor : motors_) {
        if ((motor.id + 0x200) == static_cast<int>(rx_can_id)) {
            uint16_t raw_angle = (uint16_t(rxData[0]) << 8) | rxData[1];
            motor.data.q = static_cast<double>(raw_angle) / 8192.0 * 2.0 * M_PI;

            int16_t raw_vel = (int16_t)((rxData[2] << 8) | rxData[3]);
            if (raw_vel & (1 << 15)) raw_vel -= (1 << 16);
            motor.data.dq = static_cast<double>(raw_vel) / motor.gear_ratio * 2.0 * M_PI / 60.0;

            int16_t raw_current = (int16_t)((rxData[4] << 8) | rxData[5]);
            if (raw_current & (1 << 15)) raw_current -= (1 << 16);
            motor.data.tau = static_cast<double>(raw_current) * torque_const_M3508 / 1000.0;

            motor.data.temp = static_cast<double>(rxData[6]);
        }
    }
    
    // now = std::chrono::steady_clock::now();
    // us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
    // std::cout << "t_recv: " << us << std::endl;

    return true;
}

