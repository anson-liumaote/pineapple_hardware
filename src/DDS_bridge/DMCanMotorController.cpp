// CanMotorController.cpp — SerialPort 版本（不依賴 usb2can / damiao 類別）

#include "DMCanMotorController.h"
#include "u2can/SerialPortCAN.h"           // 改成只用這個
#include <algorithm>
#include <cstdio>
#include <cmath>
#include <chrono>
#include <thread>
#include <array>
#include <cstring>

// ====== 參考 damiao.h 的封包格式（精簡拷貝，不需要包含 damiao.h） ======
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
    uint8_t  len            = 0x08;         // 資料長度
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

// ====== 你的 class 原樣保留，僅替換通訊底層 ======

DMCanMotorController::DMCanMotorController(const char* device_path, std::initializer_list<int> motor_ids)
    : device_path_(device_path), dev_(-1)
{
    for (int id : motor_ids) {
        motors_.emplace_back(id);
    }

    // 改成走 SerialPort
    // 這裡假設你的 USB-CAN 轉串口裝置出現在 device_path_（例如 /dev/ttyACM0）
    // 波特率依照你的 USB-CAN 韌體設定，常見 921600；如需修改請改這行
    try {
        serial_ = std::make_shared<SerialPortCAN>(device_path_, B921600);
        dev_ = 1; // 只作為 "已開啟" 的旗標
    } catch (...) {
        printf("Failed to open serial port: %s\n", device_path_);
        dev_ = -1;
    }
}

DMCanMotorController::~DMCanMotorController() {
    // SerialPort 在 shared_ptr 析構中自動 close
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

// 將 int16 轉兩個 byte（與你原本一致）
void DMCanMotorController::int_to_high_low_bytes(int16_t value, uint8_t& high, uint8_t& low) {
    high = static_cast<uint8_t>((value >> 8) & 0xFF);
    low  = static_cast<uint8_t>( value       & 0xFF);
}

// 小工具：發送一個 CAN 資料帧（經由 SerialPort -> USB-CAN）
bool DMCanMotorController::send_can_std_8(uint32_t can_id, const uint8_t data[8]) {
    if (!serial_) return false;
    CAN_Send_Frame frame;
    frame.modify(can_id, data);
    ssize_t n = serial_->send(reinterpret_cast<const uint8_t*>(&frame), sizeof(frame));
    return (n == static_cast<ssize_t>(sizeof(frame)));
}

// 小工具：嘗試讀一個可解析的 CAN 回包（阻塞時間由 SerialPort 的 timeout 控制）
bool DMCanMotorController::recv_can_frame(uint32_t& out_can_id, uint8_t out_data[8]) {
    if (!serial_) return false;

    CAN_Receive_Frame rx{};
    // 參考 SerialPort.h 的收包：以 0xAA 為起始頭，長度給整個結構
    // （SerialPort::recv 會自行做緩衝與頭對齊）
    serial_->recv(reinterpret_cast<uint8_t*>(&rx), 0xAA, sizeof(CAN_Receive_Frame));

    // 基本檢查（對應 damiao.h 風格）
    if (rx.FrameHeader != 0xAA)           return false;
    if (rx.frameEnd   != 0x55)            return false;
    if (rx.CMD        != 0x11)            return false;  // 0x11: receive success
    if (rx.canDataLen  > 8)               return false;  // 我們只收 0~8
    // 取出
    out_can_id = rx.canId;
    std::memcpy(out_data, rx.canData, 8);
    return true;
}

bool DMCanMotorController::sendMotorCurrents() {
    if (motors_.size() < 2 || dev_ < 0) return false;
    constexpr float torque_const_M3508 = 0.246f;

    // === 和你原本一樣：PD + feedforward ===
    float motor1_tau = static_cast<float>(
        motors_[0].cmd.kd * (motors_[0].cmd.dq - motors_[0].data.dq) +
        motors_[0].cmd.tau
    );
    float motor2_tau = static_cast<float>(
        motors_[1].cmd.kd * (motors_[1].cmd.dq - motors_[1].data.dq) +
        motors_[1].cmd.tau
    );

    // 你的 1.2 放大與限幅保留
    motor1_tau *= 1.2f;
    motor2_tau *= 1.2f;
    motor1_tau = std::clamp(motor1_tau, -10.0f, 10.0f);
    motor2_tau = std::clamp(motor2_tau, -10.0f, 10.0f);

    // 扭矩->電流->指令縮放（保留原計算）
    float motor1_current = std::clamp(motor1_tau / torque_const_M3508, -20.0f, 20.0f);
    float motor2_current = std::clamp(motor2_tau / torque_const_M3508, -20.0f, 20.0f);

    int16_t motor1_cmd = static_cast<int16_t>(motor1_current * 16384 / 20);
    int16_t motor2_cmd = static_cast<int16_t>(motor2_current * 16384 / 20);

    uint8_t m1h, m1l, m2h, m2l;
    int_to_high_low_bytes(motor1_cmd, m1h, m1l);
    int_to_high_low_bytes(motor2_cmd, m2h, m2l);

    // === 送出到你原本的 DJI/C620/C610 標準：ID=0x200，資料 8 bytes ===
    // tx 格式：M1_H, M1_L, M2_H, M2_L, 其他清零（和你原本相同）
    uint8_t txData[8] = {m1h, m1l, m2h, m2l, 0x00, 0x00, 0x00, 0x00};

    // 用 SerialPort 打包成 CAN_Send_Frame 寄送
    if (!send_can_std_8(/*can_id*/0x200, txData)) {
        printf("Failed to send CAN frame (serial)\n");
        return false;
    }

    // === 接收：用與你原本一樣的解析邏輯 ===
    // 這裡嘗試接一帧（如果你的 USB-CAN 會回回送資料），也可在 runLoop 裡多收幾帧
    uint32_t rx_can_id = 0;
    uint8_t  rxData[8] = {0};
    if (!recv_can_frame(rx_can_id, rxData)) {
        // 視需求：有些 USB-CAN 不會回 CAN 收到的回包，可以選擇不視為錯誤
        // printf("Failed to receive CAN frame (serial)\n");
        return true; // 不把它當 error，中斷會卡住
    }

    // === 寫入 motor.data（保留你的解析公式） ===
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

    return true;
}

void DMCanMotorController::runLoop(std::atomic<bool>& running) {
    using clock = std::chrono::steady_clock;
    int loop_count = 0;
    auto last_time = clock::now();

    while (running) {
        sendMotorCurrents();
        std::this_thread::sleep_for(std::chrono::microseconds(100));

        // 若你要顯示迴圈頻率，解除下面註解即可
        // ++loop_count;
        // auto now = clock::now();
        // auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_time).count();
        // if (elapsed >= 1) {
        //     printf("CAN loop frequency: %d Hz\n", loop_count);
        //     loop_count = 0;
        //     last_time = now;
        // }
    }
}
