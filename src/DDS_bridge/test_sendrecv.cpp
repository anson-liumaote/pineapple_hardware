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
std::atomic<double> l_thigh_action{0.0}; 
std::atomic<double> l_calf_action{0.0};
std::atomic<double> l_wheel_action{0.0};
std::atomic<double> r_thigh_action{0.0}; 
std::atomic<double> r_calf_action{0.0};
std::atomic<double> r_wheel_action{0.0};
std::atomic<double> l_thigh_pos{0.0}; 
std::atomic<double> l_calf_pos{0.0};
std::atomic<double> l_wheel_vel{0.0};
std::atomic<double> r_thigh_pos{0.0}; 
std::atomic<double> r_calf_pos{0.0};
std::atomic<double> r_wheel_vel{0.0};


void LowStateHandler(const void* msg) {
    const unitree_go::msg::dds_::LowState_& state = *static_cast<const unitree_go::msg::dds_::LowState_*>(msg);

    // Print motor 0 state

    // for (int i = 0; i < 6; ++i) {
    //     if (i==2){
    //         std::cout << "Motor " << i
    //             << " q: " << state.motor_state()[i].q()
    //             << " dq: " << state.motor_state()[i].dq()
    //             << " tau_est: " << state.motor_state()[i].tau_est()
    //             << std::endl;
    //     }
    // }

    // Print IMU data
    // std::cout << "IMU Quaternion: [";
    // for (int i = 0; i < 4; ++i) {
    //     std::cout << state.imu_state().quaternion()[i];
    //     if (i < 3) std::cout << ", ";
    // }
    // std::cout << "]" << std::endl;

    // std::cout << "IMU RPY: [";
    // for (int i = 0; i < 3; ++i) {
    //     std::cout << state.imu_state().rpy()[i];
    //     if (i < 2) std::cout << ", ";
    // }
    // std::cout << "]" << std::endl;

    std::cout << "IMU Gyro: [";
    for (int i = 0; i < 3; ++i) {
        std::cout << state.imu_state().gyroscope()[i];
        if (i < 2) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    // std::cout << "IMU Accel: [";
    // for (int i = 0; i < 3; ++i) {
    //     std::cout << state.imu_state().accelerometer()[i];
    //     if (i < 2) std::cout << ", ";
    // }
    // std::cout << "]" << std::endl;

    project_gravity_x = 2 * (-state.imu_state().quaternion()[1] * state.imu_state().quaternion()[3] + state.imu_state().quaternion()[0] * state.imu_state().quaternion()[2]);
    project_gravity_y = -2 * (state.imu_state().quaternion()[2] * state.imu_state().quaternion()[3] + state.imu_state().quaternion()[0] * state.imu_state().quaternion()[1]);
    project_gravity_z = 1 - 2 * (state.imu_state().quaternion()[0] * state.imu_state().quaternion()[0] + state.imu_state().quaternion()[3] * state.imu_state().quaternion()[3]);

    ang_vel_x = state.imu_state().gyroscope()[0];
    ang_vel_y = state.imu_state().gyroscope()[1];
    ang_vel_z = state.imu_state().gyroscope()[2];

    l_thigh_pos = state.motor_state()[0].q();
    l_calf_pos = state.motor_state()[1].q();
    l_wheel_vel = state.motor_state()[2].dq();
    r_thigh_pos = state.motor_state()[3].q();
    r_calf_pos = state.motor_state()[4].q();
    r_wheel_vel = state.motor_state()[5].dq();

}

void LowCmdHandler(const void* msg) {

    const unitree_go::msg::dds_::LowCmd_& cmd = *static_cast<const unitree_go::msg::dds_::LowCmd_*>(msg);

    if (cmd.motor_cmd().size() < 5) {
        std::cerr << "[ERROR] motor cmd size invalid \n";
        return;
    }

    std::cout << "l wheel action: " << cmd.motor_cmd()[2].q() << "r wheel action: " << cmd.motor_cmd()[5].q()
              << std::endl;

    l_thigh_action = cmd.motor_cmd()[0].q();
    l_calf_action = cmd.motor_cmd()[1].q();
    l_wheel_action = cmd.motor_cmd()[2].dq();
    r_thigh_action = cmd.motor_cmd()[3].q();
    r_calf_action = cmd.motor_cmd()[4].q();
    r_wheel_action = cmd.motor_cmd()[5].dq();

}

int main() {
    // Initialize DDS
    ChannelFactory::Instance()->Init(1, "lo");

    // Publisher for LowCmd
    // auto low_cmd_pub = std::make_shared<ChannelPublisher<unitree_go::msg::dds_::LowCmd_>>(TOPIC_LOWCMD);
    // low_cmd_pub->InitChannel();
    
    // Subscriber for LowCmd
    auto low_cmd_sub = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::LowCmd_>>(TOPIC_LOWCMD);
    low_cmd_sub->InitChannel(LowCmdHandler, 1);

    // Subscriber for LowState
    auto low_state_sub = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::LowState_>>(TOPIC_LOWSTATE);
    low_state_sub->InitChannel(LowStateHandler, 1);

    std::ofstream log("imu_test.csv");
    log << "time,ang_vel_x,ang_vel_y,ang_vel_z,project_gravity_x,project_gravity_y,project_gravity_z,l_thigh_action,l_calf_action,l_wheel_action,r_thigh_action,r_calf_action,r_wheel_action,l_thigh_pos,l_calf_pos,l_wheel_vel,r_thigh_pos,r_calf_pos,r_wheel_vel" << "\n";

    double step_time = 5.0; // seconds before step
    double step_value = 3.0;
    double dt = 0.002; // 2ms loop
    double t = 0.0;

    while (t < 1.0) { // Run for 1 seconds
        unitree_go::msg::dds_::LowCmd_ cmd;
        auto& motors = cmd.motor_cmd();
        // for (size_t i = 0; i < 6; ++i) {
        //     motors[i].dq() = 0.0;
        //     motors[i].tau() = 0.0;
        //     motors[i].kp() = 0.0;
        //     motors[i].kd() = 0.0;
        // }

        // Step input for dq on motor 2
        // double dq_cmd = (t >= step_time) ? step_value : 0.0;
        // double amp = 1.5;
        // double freq = 10.0;
        // double dq_cmd = amp * std::sin(2.0 * M_PI * t * freq);
        motors[2].dq() = -3.0;
        motors[2].kd() = 1.5;

        // motors[5].kd() = 0.3;

        // low_cmd_pub->Write(cmd);

        // Log time, command dq, and state dq
        log << t << "," << ang_vel_x.load() << "," << ang_vel_y.load() << "," << ang_vel_z.load() << "," << project_gravity_x.load() << "," << project_gravity_y.load() << "," << project_gravity_z.load() << "," << l_thigh_action.load() << "," << l_calf_action.load() <<"," << l_wheel_action.load() <<  "," << r_thigh_action.load() << "," << r_calf_action.load() << "," << r_wheel_action.load() << "," << l_thigh_pos.load() << "," << l_calf_pos.load() << "," << l_wheel_vel.load() << "," << r_thigh_pos.load() << "," << r_calf_pos.load() << "," << r_wheel_vel.load() << "\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000)));
        t += dt;
    }
    log.close();
    return 0;
}
