#include "DDS_bridge.h"
#include <thread>
#include <chrono>
#include <iostream>

DDSBridge::DDSBridge(MotorController& hip, MotorController& thigh, MotorController& calf, DMCanMotorController& can, ImuSharedData* imuData)
    : controllerHip_(hip), controllerThigh_(thigh), controllerCalf_(calf), canController_(can), imuData_(imuData)
{
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

    if (cmd.motor_cmd().size() < 8) {
        std::cerr << "[ERROR] motor cmd size invalid \n";
        return;
    }
    // Hip
    controllerHip_.setJointCommand(0, -cmd.motor_cmd()[0].q(), -cmd.motor_cmd()[0].dq(), -cmd.motor_cmd()[0].tau(), cmd.motor_cmd()[0].kp(), cmd.motor_cmd()[0].kd());
    controllerHip_.setJointCommand(1, -cmd.motor_cmd()[4].q(), -cmd.motor_cmd()[4].dq(), -cmd.motor_cmd()[4].tau(), cmd.motor_cmd()[4].kp(), cmd.motor_cmd()[4].kd());
    // Thigh
    controllerThigh_.setJointCommand(0, cmd.motor_cmd()[1].q(), cmd.motor_cmd()[1].dq(), cmd.motor_cmd()[1].tau(), cmd.motor_cmd()[1].kp(), cmd.motor_cmd()[1].kd());
    controllerThigh_.setJointCommand(1, -cmd.motor_cmd()[5].q(), -cmd.motor_cmd()[5].dq(), -cmd.motor_cmd()[5].tau(), cmd.motor_cmd()[5].kp(), cmd.motor_cmd()[5].kd());
    // Calf
    controllerCalf_.setJointCommand(0, cmd.motor_cmd()[2].q(), cmd.motor_cmd()[2].dq(), cmd.motor_cmd()[2].tau(), cmd.motor_cmd()[2].kp(), cmd.motor_cmd()[2].kd());
    controllerCalf_.setJointCommand(1, -cmd.motor_cmd()[6].q(), -cmd.motor_cmd()[6].dq(), -cmd.motor_cmd()[6].tau(), -cmd.motor_cmd()[6].kp(), cmd.motor_cmd()[6].kd());
    // Left wheel
    canController_.setCommand(1, cmd.motor_cmd()[3].dq(), cmd.motor_cmd()[3].tau(), cmd.motor_cmd()[3].kd());
    // Right thigh
    // controllerRight_.setJointCommand(4, -cmd.motor_cmd()[3].q(), -cmd.motor_cmd()[3].dq(), -cmd.motor_cmd()[3].tau(), cmd.motor_cmd()[3].kp(), cmd.motor_cmd()[3].kd());
    // Right knee
    // controllerRight_.setJointCommand(5, -cmd.motor_cmd()[4].q(), -cmd.motor_cmd()[4].dq(), -cmd.motor_cmd()[4].tau(), cmd.motor_cmd()[4].kp(), cmd.motor_cmd()[4].kd());
    // Right wheel
    canController_.setCommand(2, -cmd.motor_cmd()[7].dq(), -cmd.motor_cmd()[7].tau(), cmd.motor_cmd()[7].kd());

}

void DDSBridge::PublishLowState()
{
    unitree_go::msg::dds_::LowState_ msg;
    std::size_t n = msg.motor_state().size();
    if (n < 8) {
        std::cerr << "Error: msg.motor_state() size is " << n << ", expected at least 6!" << std::endl;
        return;
    }

    // mapping motor states
    if (controllerHip_.getMotors().size() > 0) {
        const auto& m = controllerHip_.getMotors()[0];
        msg.motor_state()[0].q() = -m.getJointPosition();
        msg.motor_state()[0].dq() = -m.getJointVelocity();
        msg.motor_state()[0].tau_est() = -m.getJointTorque();
    }
    if (controllerHip_.getMotors().size() > 1) {
        const auto& m = controllerHip_.getMotors()[1];
        msg.motor_state()[4].q() = -m.getJointPosition();
        msg.motor_state()[4].dq() = -m.getJointVelocity();
        msg.motor_state()[4].tau_est() = -m.getJointTorque();
    }
    if (controllerThigh_.getMotors().size() > 0) {
        const auto& m = controllerThigh_.getMotors()[0];
        msg.motor_state()[1].q() = m.getJointPosition();
        msg.motor_state()[1].dq() = m.getJointVelocity();
        msg.motor_state()[1].tau_est() = m.getJointTorque();
    }
    if (controllerThigh_.getMotors().size() > 1) {
        const auto& m = controllerThigh_.getMotors()[1];
        msg.motor_state()[5].q() = -m.getJointPosition();
        msg.motor_state()[5].dq() = -m.getJointVelocity();
        msg.motor_state()[5].tau_est() = -m.getJointTorque();
    }
    if (controllerCalf_.getMotors().size() > 0) {
        const auto& m = controllerCalf_.getMotors()[0];
        msg.motor_state()[2].q() = m.getJointPosition();
        msg.motor_state()[2].dq() = m.getJointVelocity();
        msg.motor_state()[2].tau_est() = m.getJointTorque();
    }
    if (controllerCalf_.getMotors().size() > 1) {
        const auto& m = controllerCalf_.getMotors()[1];
        msg.motor_state()[6].q() = -m.getJointPosition();
        msg.motor_state()[6].dq() = -m.getJointVelocity();
        msg.motor_state()[6].tau_est() = -m.getJointTorque();
    }
    // CAN motors
    if (canController_.getMotors().size() > 0) {
        const auto& m = canController_.getMotors()[0];
        msg.motor_state()[3].q() = m.data.q;
        msg.motor_state()[3].dq() = m.data.dq;
        msg.motor_state()[3].tau_est() = m.data.tau;
    }
    if (canController_.getMotors().size() > 1) {
        const auto& m = canController_.getMotors()[1];
        msg.motor_state()[7].q() = -m.data.q;
        msg.motor_state()[7].dq() = -m.data.dq;
        msg.motor_state()[7].tau_est() = -m.data.tau;
    }

    // --- IMU data assignment ---
    if (imuData_) {
        std::lock_guard<std::mutex> lock(imuData_->mtx);
        for (int i = 0; i < 4; ++i)
            msg.imu_state().quaternion()[i] = imuData_->quaternion[i];
        for (int i = 0; i < 3; ++i) {
            // msg.imu_state().rpy()[i] = imuData_->rpy[i];
            msg.imu_state().gyroscope()[i] = imuData_->gyro[i];
            msg.imu_state().accelerometer()[i] = imuData_->accel[i];
        }
    }

    low_state_puber_->Write(msg);
}
