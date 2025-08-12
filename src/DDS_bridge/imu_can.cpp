#include "ImuSharedData.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>

#define ACCEL_CAN_MAX (58.8f)
#define ACCEL_CAN_MIN (-58.8f)
#define GYRO_CAN_MAX (34.88f)
#define GYRO_CAN_MIN (-34.88f)
#define PITCH_CAN_MAX (90.0f)
#define PITCH_CAN_MIN (-90.0f)
#define ROLL_CAN_MAX (180.0f)
#define ROLL_CAN_MIN (-180.0f)
#define YAW_CAN_MAX (180.0f)
#define YAW_CAN_MIN (-180.0f)
#define TEMP_MIN (0.0f)
#define TEMP_MAX (60.0f)
#define Quaternion_MIN (-1.0f)
#define Quaternion_MAX (1.0f)

int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

bool send_request(int socket_fd, uint8_t canid_l, uint8_t canid_h, uint8_t reg)
{
    struct can_frame frame{};
    frame.can_id = 0x6FF;
    frame.can_dlc = 4;
    frame.data[0] = canid_l;
    frame.data[1] = canid_h;
    frame.data[2] = reg;
    frame.data[3] = 0xCC;
    return write(socket_fd, &frame, sizeof(frame)) == sizeof(frame);
}

void IMU_UpdateAccel(ImuSharedData* imu, uint8_t* pData)
{
    uint16_t accel[3];
    accel[0] = pData[3] << 8 | pData[2];
    accel[1] = pData[5] << 8 | pData[4];
    accel[2] = pData[7] << 8 | pData[6];
    imu->accel[0] = uint_to_float(accel[0], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
    imu->accel[1] = uint_to_float(accel[1], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
    imu->accel[2] = uint_to_float(accel[2], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
    std::cout << "[IMU] Accel: "
              << imu->accel[0] << ", "
              << imu->accel[1] << ", "
              << imu->accel[2] << std::endl;
}

void IMU_UpdateGyro(ImuSharedData* imu, uint8_t* pData)
{
    uint16_t gyro[3];
    gyro[0] = pData[3] << 8 | pData[2];
    gyro[1] = pData[5] << 8 | pData[4];
    gyro[2] = pData[7] << 8 | pData[6];
    imu->gyro[0] = uint_to_float(gyro[0], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
    imu->gyro[1] = uint_to_float(gyro[1], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
    imu->gyro[2] = uint_to_float(gyro[2], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
    std::cout << "[IMU] Gyro: "
              << imu->gyro[0] << ", "
              << imu->gyro[1] << ", "
              << imu->gyro[2] << std::endl;
}

void IMU_UpdateEuler(ImuSharedData* imu, uint8_t* pData)
{
    int euler[3];
    euler[0] = pData[3] << 8 | pData[2];
    euler[1] = pData[5] << 8 | pData[4];
    euler[2] = pData[7] << 8 | pData[6];
    imu->rpy[0] = uint_to_float(euler[0], PITCH_CAN_MIN, PITCH_CAN_MAX, 16);
    imu->rpy[1] = uint_to_float(euler[1], YAW_CAN_MIN, YAW_CAN_MAX, 16);
    imu->rpy[2] = uint_to_float(euler[2], ROLL_CAN_MIN, ROLL_CAN_MAX, 16);
    std::cout << "[IMU] Euler (RPY): "
              << imu->rpy[0] << ", "
              << imu->rpy[1] << ", "
              << imu->rpy[2] << std::endl;
}

void IMU_UpdateQuaternion(ImuSharedData* imu, uint8_t* pData)
{
    int w = pData[1] << 6 | ((pData[2] & 0xF8) >> 2);
    int x = (pData[2] & 0x03) << 12 | (pData[3] << 4) | ((pData[4] & 0xF0) >> 4);
    int y = (pData[4] & 0x0F) << 10 | (pData[5] << 2) | ((pData[6] & 0xC0) >> 6);
    int z = (pData[6] & 0x3F) << 8 | pData[7];
    imu->quaternion[0] = uint_to_float(w, Quaternion_MIN, Quaternion_MAX, 14);
    imu->quaternion[1] = uint_to_float(x, Quaternion_MIN, Quaternion_MAX, 14);
    imu->quaternion[2] = uint_to_float(y, Quaternion_MIN, Quaternion_MAX, 14);
    imu->quaternion[3] = uint_to_float(z, Quaternion_MIN, Quaternion_MAX, 14);
    std::cout << "[IMU] Quaternion: "
              << imu->quaternion[0] << ", "
              << imu->quaternion[1] << ", "
              << imu->quaternion[2] << ", "
              << imu->quaternion[3] << std::endl;
}

void IMU_UpdateData(ImuSharedData* imu, uint8_t* pData)
{
    switch (pData[0]) {
        case 1:
            IMU_UpdateAccel(imu, pData);
            break;
        case 2:
            IMU_UpdateGyro(imu, pData);
            break;
        case 3:
            IMU_UpdateEuler(imu, pData);
            break;
        case 4:
            IMU_UpdateQuaternion(imu, pData);
            break;
    }
}

void imuCANThread(std::atomic<bool>& running, ImuSharedData* imuData)
{
    int socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd < 0) {
        std::cerr << "Failed to open CAN socket\n";
        return;
    }

    struct ifreq ifr{};
    std::strcpy(ifr.ifr_name, "can0");
    ioctl(socket_fd, SIOCGIFINDEX, &ifr);

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "Failed to bind CAN socket\n";
        return;
    }

    uint16_t can_id = 1;
    uint8_t can_l = can_id & 0xFF;
    uint8_t can_h = (can_id >> 8) & 0xFF;

    // Frequency measurement variables
    size_t loop_count = 0;
    auto last_time = std::chrono::steady_clock::now();

    while (running) {
        for (uint8_t reg = 0x01; reg <= 0x04; ++reg) {
            auto t_send_start = std::chrono::steady_clock::now();
            send_request(socket_fd, 0x01, 0x00, reg);
            send_request(socket_fd, 0x02, 0x00, reg);
            // send_request(socket_fd, can_l, can_h, 0x02);
            // send_request(socket_fd, can_l, can_h, 0x03);
            // send_request(socket_fd, can_l, can_h, 0x04);
            auto t_send_end = std::chrono::steady_clock::now();

            auto t_read_start = std::chrono::steady_clock::now();
            struct can_frame frame1;
            struct can_frame frame2;
            int nbytes1 = read(socket_fd, &frame1, sizeof(frame1));
            // int nbytes2 = read(socket_fd, &frame2, sizeof(frame2));
            // int nbytes3 = read(socket_fd, &frame3, sizeof(frame3));
            // int nbytes4 = read(socket_fd, &frame4, sizeof(frame4));
            auto t_read_end = std::chrono::steady_clock::now();

            auto send_duration_us = std::chrono::duration_cast<std::chrono::microseconds>(t_send_end - t_send_start).count();
            auto read_duration_us = std::chrono::duration_cast<std::chrono::microseconds>(t_read_end - t_read_start).count();

            if (nbytes1 >= 0 && frame1.can_dlc == 8) {
                std::lock_guard<std::mutex> lock(imuData->mtx);
                IMU_UpdateData(imuData, frame1.data);
            }
            // if (nbytes2 >= 0 && frame2.can_dlc == 8) {
            //     std::lock_guard<std::mutex> lock(imuData->mtx);
            //     IMU_UpdateData(imuData, frame2.data);
            // }
            // if (nbytes3 >= 0 && frame3.can_dlc == 8) {
            //     std::lock_guard<std::mutex> lock(imuData->mtx);
            //     IMU_UpdateData(imuData, frame3.data);
            // }
            // if (nbytes4 >= 0 && frame4.can_dlc == 8) {
            //     std::lock_guard<std::mutex> lock(imuData->mtx);
            //     IMU_UpdateData(imuData, frame4.data);
            // }

            std::cout << "[IMU_CAN] send_request time: " << send_duration_us << " us, "
                      << "read time: " << read_duration_us << " us" << std::endl;

            ++loop_count;
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_time).count();
            if (elapsed >= 1) {
                std::cout << "[IMU_CAN] Loop frequency: " << loop_count / elapsed << " Hz" << std::endl;
                loop_count = 0;
                last_time = now;
            }
        }
        
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    close(socket_fd);
}

// Usage example in main()
// std::atomic<bool> running(true);
// ImuSharedData imuData;
// std::thread imuThread(imuCANThread, std::ref(running), &imuData);
// ...
// running = false; imuThread.join();
