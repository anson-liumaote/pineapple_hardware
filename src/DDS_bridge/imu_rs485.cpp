#include "ImuSharedData.h"
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <vector>

#define FRAME_HEADER 0x55
#define FRAME_FLAG   0xAA
#define FRAME_TAIL   0x0A

#define FRAME_LENGTH_QUATERNION 23
#define FRAME_LENGTH_OTHER 19

// CRC16 查表（根據使用說明書）
static const uint16_t CRC16_table[256] = { 0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108, 0x9129,
    0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318,
    0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485, 0xA56A, 0xB54B,
    0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4, 0xB75B, 0xA77A,
    0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823, 0xC9CC, 0xD9ED,
    0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC,
    0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F,
    0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70, 0xFF9F, 0xEFBE,
    0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1,
    0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 0x02B1, 0x1290,
    0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D, 0x34E2, 0x24C3,
    0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C, 0x26D3, 0x36F2,
    0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB, 0x5844, 0x4865,
    0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A, 0x4A75, 0x5A54,
    0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07,
    0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36,
    0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0 };

uint16_t Get_CRC16(uint8_t *ptr, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        uint8_t index = (crc >> 8) ^ ptr[i];
        crc = ((crc << 1) ^ CRC16_table[index]);
    }
    return crc;
}

int setupSerial(const char* portname) {
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("open serial port");
        return -1;
    }

    struct termios tty {};
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, B921600);
    cfsetispeed(&tty, B921600);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        close(fd);
        return -1;
    }

    return fd;
}

void processIMUFrame(ImuSharedData* imu, const uint8_t* data, size_t length) {
    uint8_t reg_id = data[3];
    if (reg_id == 0x01 || reg_id == 0x02 || reg_id == 0x03) {
        float values[3];
        std::memcpy(&values[0], &data[4], 4);
        std::memcpy(&values[1], &data[8], 4);
        std::memcpy(&values[2], &data[12], 4);

        std::lock_guard<std::mutex> lock(imu->mtx);
        if (reg_id == 0x01) {
            imu->accel[0] = values[0];
            imu->accel[1] = values[1];
            imu->accel[2] = values[2];
            std::cout << "[IMU] Accel: " << values[0] << ", " << values[1] << ", " << values[2] << std::endl;
            auto now = std::chrono::steady_clock::now();
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
            std::cout << "[IMU] data published at " << ms << " ms" << std::endl;
        } else if (reg_id == 0x02) {
            imu->gyro[0] = values[0];
            imu->gyro[1] = values[1];
            imu->gyro[2] = values[2];
            std::cout << "[IMU] Gyro: " << values[0] << ", " << values[1] << ", " << values[2] << std::endl;
            auto now = std::chrono::steady_clock::now();
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
            std::cout << "[IMU] data published at " << ms << " ms" << std::endl;
        } else if (reg_id == 0x03) {
            imu->rpy[0] = values[0];
            imu->rpy[1] = values[1];
            imu->rpy[2] = values[2];
            std::cout << "[IMU] Euler (RPY): " << values[0] << ", " << values[1] << ", " << values[2] << std::endl;
            auto now = std::chrono::steady_clock::now();
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
            std::cout << "[IMU] data published at " << ms << " ms" << std::endl;
        }
    } else if (reg_id == 0x04 && length == FRAME_LENGTH_QUATERNION) {
        float quat[4];
        std::memcpy(&quat[0], &data[4], 4);
        std::memcpy(&quat[1], &data[8], 4);
        std::memcpy(&quat[2], &data[12], 4);
        std::memcpy(&quat[3], &data[16], 4);

        // std::lock_guard<std::mutex> lock(imu->mtx);
        imu->quaternion[0] = quat[0];
        imu->quaternion[1] = quat[1];
        imu->quaternion[2] = quat[2];
        imu->quaternion[3] = quat[3];

        std::cout << "[IMU] Quaternion: " << quat[0] << ", " << quat[1] << ", " << quat[2] << ", " << quat[3] << std::endl;
        auto now = std::chrono::steady_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        std::cout << "[IMU] data published at " << ms << " ms" << std::endl;
    }
}

void imuRS485Thread(std::atomic<bool>& running, ImuSharedData* imuData, const char* portname) {
    int fd = setupSerial(portname);
    if (fd < 0) return;

    // 開啟 IMU 輸出所需的初始化指令
    // const uint8_t init_cmds[][4] = {
    //     {0xAA, 0x06, 0x01, 0x0D}, // 進入設定模式
    //     {0xAA, 0x02, 0x06, 0x0D}, // 設定輸出頻率為1000hz
    //     {0xAA, 0x01, 0x14, 0x0D}, // 開啟加速度
    //     {0xAA, 0x01, 0x15, 0x0D}, // 開啟角速度
    //     {0xAA, 0x01, 0x16, 0x0D}, // 開啟歐拉角
    //     {0xAA, 0x01, 0x17, 0x0D},  // 開啟四元數
    //     {0xAA, 0x03, 0x01, 0x0D}, //保存參數
    //     {0xAA, 0x06, 0x00, 0x0D}, // 退出設定模式
    // };
    // for (const auto& cmd : init_cmds) {
    //     write(fd, cmd, sizeof(cmd));
    //     usleep(10000); // 10ms 間隔
    // }
    // if (fd < 0) return;

    std::vector<uint8_t> buffer;
    // Frequency measurement variables
    size_t loop_count = 0;
    auto last_time = std::chrono::steady_clock::now();

    while (running) {
        uint8_t byte;
        // std::cout << "[IMU] waiting for data " << std::endl;
        if (read(fd, &byte, 1) > 0) {
            buffer.push_back(byte);
            auto now = std::chrono::steady_clock::now();
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
            // std::cout << "[IMU] buffer update at " << ms << " ms" << std::endl;
            std::cout << "[IMU] buffer size: " << buffer.size() << " at " << ms << std::endl;
            // std::cout << "pass 1: " << static_cast<int>(byte) << std::endl;

            if (buffer.size() >= 5) {
                if (buffer[0] != FRAME_HEADER || buffer[1] != FRAME_FLAG) {
                    buffer.clear();
                    continue;
                }
                // std::cout << "pass 2" <<std::endl;
                uint8_t can_id = buffer[2]; 
                uint8_t reg_id = buffer[3];
                size_t frame_len = (reg_id == 0x04) ? FRAME_LENGTH_QUATERNION : FRAME_LENGTH_OTHER;

                // if (buffer.size() >= frame_len) std::cout << "pass 3-1" <<std::endl;
                // if (buffer[frame_len - 1] == FRAME_TAIL) std::cout << "pass 3-2" <<std::endl;
                // else std::cout << static_cast<int>(buffer[frame_len - 1]) <<std::endl;

                if (buffer.size() >= frame_len && buffer[frame_len - 1] == FRAME_TAIL) {
                    // std::cout << "pass 3" <<std::endl;
                    uint16_t crc_recv = buffer[frame_len - 3] | (buffer[frame_len - 2] << 8);
                    uint16_t crc_calc = Get_CRC16(buffer.data(), frame_len - 3);
                    if (crc_recv == crc_calc) {
                        // 新增：依照讀到的 can_id 寫入 imuData->id
                        {
                            // std::lock_guard<std::mutex> lock(imuData->mtx);
                            imuData->id = static_cast<int>(can_id);
                        }
                        processIMUFrame(imuData, buffer.data(), frame_len);
                        // ++loop_count;
                        // auto now = std::chrono::steady_clock::now();
                        // auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_time).count(); 
                        // std::cout << elapsed << std::endl;
                        // std::cout << "[IMU] ID: " << static_cast<int>(can_id) << std::endl;
                        // std::cout << "[IMU: " << portname << "] Loop frequency: " << loop_count / elapsed << " Hz" << std::endl;

                        // if (elapsed >= 1.0) {
                        //     // std::cout << "[IMU: " << portname << "] Loop frequency: " << loop_count / elapsed << " Hz" << std::endl;
                        //     loop_count = 0;
                        //     last_time = now;
                        // }
                    }
                    // std::cout << "[IMU] buffer size 2: " << buffer.size() << std::endl;
                    buffer.clear();
                    // std::cout << "[IMU] buffer size 3: " << buffer.size() << std::endl;
                    // std::this_thread::sleep_for(std::chrono::milliseconds(2)); // 0.5 ms
                }
            } else if (buffer.size() > 128) {
                buffer.clear();
            }
            // std::this_thread::sleep_for(std::chrono::microseconds(1));
        }
        
    }
    close(fd);
}

// 使用範例：
// std::atomic<bool> running(true);
// ImuSharedData imuData;
// std::thread imuThread(imuRS485Thread, std::ref(running), &imuData);
// ...
// running = false; imuThread.join();
