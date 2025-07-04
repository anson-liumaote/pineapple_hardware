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

std::atomic<double> last_motor2_dq_state{0.0}; // For thread-safe sharing

void LowStateHandler(const void* msg) {
    const unitree_go::msg::dds_::LowState_& state = *static_cast<const unitree_go::msg::dds_::LowState_*>(msg);

    // Print motor 0 state

    for (int i = 0; i < 6; ++i) {
        std::cout << "Motor " << i
              << " q: " << state.motor_state()[i].q()
              << " dq: " << state.motor_state()[i].dq()
              << " tau_est: " << state.motor_state()[i].tau_est()
              << std::endl;
    }

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

    last_motor2_dq_state = state.motor_state()[2].dq(); // Save dq for motor 2
}

int main() {
    // Initialize DDS
    ChannelFactory::Instance()->Init(1, "lo");

    // Publisher for LowCmd
    auto low_cmd_pub = std::make_shared<ChannelPublisher<unitree_go::msg::dds_::LowCmd_>>(TOPIC_LOWCMD);
    low_cmd_pub->InitChannel();

    // Subscriber for LowState
    auto low_state_sub = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::LowState_>>(TOPIC_LOWSTATE);
    low_state_sub->InitChannel(LowStateHandler, 1);

    std::ofstream log("dq_step_log.csv");
    log << "time,cmd_dq,state_dq\n";

    double step_time = 5.0; // seconds before step
    double step_value = 3.0;
    double dt = 0.01; // 10ms loop
    double t = 0.0;

    while (t < 3.0) { // Run for 15 seconds
        unitree_go::msg::dds_::LowCmd_ cmd;
        auto& motors = cmd.motor_cmd();
        for (size_t i = 0; i < 6; ++i) {
            motors[i].dq() = 0.0;
            motors[i].tau() = 0.0;
            motors[i].kp() = 0.0;
            motors[i].kd() = 0.0;
        }

        // Step input for dq on motor 2
        // double dq_cmd = (t >= step_time) ? step_value : 0.0;
        double dq_cmd = 3.0 * std::sin(2.0 * M_PI * t / 0.5);
        motors[2].dq() = dq_cmd;
        motors[2].kd() = 0.3;

        // motors[5].kd() = 0.3;

        low_cmd_pub->Write(cmd);

        // Log time, command dq, and state dq
        log << t << "," << dq_cmd << "," << last_motor2_dq_state.load() << "\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000)));
        t += dt;
    }
    log.close();
    return 0;
}
