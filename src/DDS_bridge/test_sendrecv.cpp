#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <fstream>
#include <atomic>
#include <cmath>

using namespace unitree::robot;

#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_LOWCMD "rt/lowcmd"

std::atomic<double> ang_vel_x{0.0}; // For thread-safe sharing
std::atomic<double> ang_vel_y{0.0};
std::atomic<double> ang_vel_z{0.0};
std::atomic<double> project_gravity_x{0.0}; 
std::atomic<double> project_gravity_y{0.0};
std::atomic<double> project_gravity_z{0.0};


void LowStateHandler(const void* msg) {
    const unitree_go::msg::dds_::LowState_& state = *static_cast<const unitree_go::msg::dds_::LowState_*>(msg);
    
    for (int i = 0; i < 1; ++i) { // 印出 IMU[1] 和 IMU[2]
        std::cout << "IMU[" << i << "] Gyro: [";
        for (int j = 0; j < 3; ++j) {
            std::cout << state.imu_state()[i].gyroscope()[j];
            if (j < 2) std::cout << ", ";
        }
        // std::cout << "] | Accel: [";
        // for (int j = 0; j < 3; ++j) {
        //     std::cout << state.imu_state()[i].accelerometer()[j];
        //     if (j < 2) std::cout << ", ";
        // }
        // std::cout << "] | Quat: [";
        // for (int j = 0; j < 4; ++j) {
        //     std::cout << state.imu_state()[i].quaternion()[j];
        //     if (j < 3) std::cout << ", ";
        // }
        std::cout << "]" << std::endl;
    }

    // project_gravity_x = 2 * (-state.imu_state().quaternion()[1] * state.imu_state().quaternion()[3] + state.imu_state().quaternion()[0] * state.imu_state().quaternion()[2]);
    // project_gravity_y = -2 * (state.imu_state().quaternion()[2] * state.imu_state().quaternion()[3] + state.imu_state().quaternion()[0] * state.imu_state().quaternion()[1]);
    // project_gravity_z = 1 - 2 * (state.imu_state().quaternion()[0] * state.imu_state().quaternion()[0] + state.imu_state().quaternion()[3] * state.imu_state().quaternion()[3]);

    // ang_vel_x = state.imu_state().gyroscope()[0];
    // ang_vel_y = state.imu_state().gyroscope()[1];
    // ang_vel_z = state.imu_state().gyroscope()[2];


}

int main() {
    // Initialize DDS
    ChannelFactory::Instance()->Init(1, "lo");

    // Subscriber for LowState
    auto low_state_sub = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::LowState_>>(TOPIC_LOWSTATE);
    low_state_sub->InitChannel(LowStateHandler, 1);

    double step_time = 5.0; // seconds before step
    double step_value = 3.0;
    double dt = 0.002; // 2ms loop
    double t = 0.0;

    while (t < 5.0) { 
        std::this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000)));
        t += dt;
    }
    // log.close();
    return 0;
}
