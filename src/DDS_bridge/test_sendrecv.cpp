#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>

using namespace unitree::robot;

#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_LOWCMD "rt/lowcmd"

void LowStateHandler(const void* msg) {
    const unitree_go::msg::dds_::LowState_& state = *static_cast<const unitree_go::msg::dds_::LowState_*>(msg);

    // Print motor 0 state
    // std::cout << "Motor 0"
    //           << " q: " << state.motor_state()[0].q()
    //           << " dq: " << state.motor_state()[0].dq()
    //           << " tau_est: " << state.motor_state()[0].tau_est()
    //           << std::endl;

    // Print IMU data
    std::cout << "IMU Quaternion: [";
    for (int i = 0; i < 4; ++i) {
        std::cout << state.imu_state().quaternion()[i];
        if (i < 3) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    std::cout << "IMU RPY: [";
    for (int i = 0; i < 3; ++i) {
        std::cout << state.imu_state().rpy()[i];
        if (i < 2) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    std::cout << "IMU Gyro: [";
    for (int i = 0; i < 3; ++i) {
        std::cout << state.imu_state().gyroscope()[i];
        if (i < 2) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    std::cout << "IMU Accel: [";
    for (int i = 0; i < 3; ++i) {
        std::cout << state.imu_state().accelerometer()[i];
        if (i < 2) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
}

int main() {
    // Initialize DDS
    ChannelFactory::Instance()->Init(0, "lo");

    // Publisher for LowCmd
    auto low_cmd_pub = std::make_shared<ChannelPublisher<unitree_go::msg::dds_::LowCmd_>>(TOPIC_LOWCMD);
    low_cmd_pub->InitChannel();

    // Subscriber for LowState
    auto low_state_sub = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::LowState_>>(TOPIC_LOWSTATE);
    low_state_sub->InitChannel(LowStateHandler, 1);

    // Publish dummy LowCmd every second
    while (true) {
        unitree_go::msg::dds_::LowCmd_ cmd;
        // Fill with dummy data
        auto& motors = cmd.motor_cmd();
        // motors.resize(6);
        for (size_t i = 0; i < 6; ++i) {
            // motors[i].q() = 0.0;
            motors[i].dq() = 0.0;
            motors[i].tau() = 0.0;
            motors[i].kp() = 0.0;
            motors[i].kd() = 0.0;
        }
        motors[0].q() = 0.3;
        motors[1].q() = -0.3;
        motors[2].q() = 0.3;
        motors[3].q() = -0.3;

        low_cmd_pub->Write(cmd);
        // std::cout << "[LowCmd] Published dummy command." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return 0;
}