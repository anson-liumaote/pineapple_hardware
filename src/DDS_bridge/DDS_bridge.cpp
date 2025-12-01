#include "DDS_bridge.h"
#include <thread>
#include <chrono>
#include <iostream>

DDSBridge::DDSBridge(ImuSharedData* imuDataArr[5])
{
    for (int i = 0; i < 5; ++i)
        imuDataArr_[i] = imuDataArr[i];
    low_state_puber_.reset(new ChannelPublisher<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    low_state_puber_->InitChannel();
    low_cmd_suber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    low_cmd_suber_->InitChannel(std::bind(&DDSBridge::LowCmdHandler, this, std::placeholders::_1), 1);
    
    lowStatePuberThreadPtr = CreateRecurrentThreadEx("lowstate", UT_CPU_ID_NONE, 2000, &DDSBridge::PublishLowState, this);
    std::cout << "[DDS_Bridge] DDS publisher and subscriber initialized." << std::endl;
}

DDSBridge::~DDSBridge() {}

void DDSBridge::LowCmdHandler(const void* msg)
{
    const unitree_go::msg::dds_::LowCmd_& cmd = *static_cast<const unitree_go::msg::dds_::LowCmd_*>(msg);

    if (cmd.motor_cmd().size() < 5) {
        std::cerr << "[ERROR] motor cmd size invalid \n";
        return;
    }

}

void DDSBridge::PublishLowState()
{
    static size_t loop_count = 0;
    static auto last_time = std::chrono::steady_clock::now();

    unitree_go::msg::dds_::LowState_ msg;
    std::size_t n = msg.motor_state().size();
    // --- IMU data assignment ---
    for (int i = 0; i < 5; ++i) {
        if (imuDataArr_[i]) {
            std::lock_guard<std::mutex> lock(imuDataArr_[i]->mtx);
            for (int j = 0; j < 4; ++j)
                msg.imu_state()[i].quaternion()[j] = imuDataArr_[i]->quaternion[j];
            for (int j = 0; j < 3; ++j) {
                msg.imu_state()[i].gyroscope()[j] = imuDataArr_[i]->gyro[j];
                msg.imu_state()[i].accelerometer()[j] = imuDataArr_[i]->accel[j];
            }

            // Print out IMU data for debug
            // std::cout << "IMU[" << i << "] quat: [";
            // for (int j = 0; j < 4; ++j) {
            //     std::cout << imuDataArr_[i]->quaternion[j];
            //     if (j < 3) std::cout << ", ";
            // }
            // std::cout << "] | gyro: [";
            // for (int j = 0; j < 3; ++j) {
            //     std::cout << imuDataArr_[i]->gyro[j];
            //     if (j < 2) std::cout << ", ";
            // }
            // std::cout << "] | accel: [";
            // for (int j = 0; j < 3; ++j) {
            //     std::cout << imuDataArr_[i]->accel[j];
            //     if (j < 2) std::cout << ", ";
            // }
            // std::cout << "]" << std::endl;
            // auto now = std::chrono::steady_clock::now();
            // auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
            // std::cout << "[DDSBridge] IMU[" << i << "] data published at " 
            //           << ms << " ms" << std::endl;
        }
    }

    low_state_puber_->Write(msg);

    // // --- Frequency measurement ---
    // ++loop_count;
    // auto now = std::chrono::steady_clock::now();
    // auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_time).count();
    // std::cout << "[DDSBridge] PublishLowState frequency: " << loop_count / elapsed << " Hz" << std::endl;
    // if (elapsed >= 1.0) {
    //     // std::cout << "[DDSBridge] PublishLowState frequency: " << loop_count / elapsed << " Hz" << std::endl;
    //     loop_count = 0;
    //     last_time = now;
    // }
}
