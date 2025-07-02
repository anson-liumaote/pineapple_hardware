#include "DDS_bridge.h"
#include <thread>
#include <chrono>
#include <iostream>

DDSBridge::DDSBridge(MotorController& left, MotorController& right, CanMotorController& can, ImuSharedData* imuData)
    : controllerLeft_(left), controllerRight_(right), canController_(can), imuData_(imuData)
{
    low_state_puber_.reset(new ChannelPublisher<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    low_state_puber_->InitChannel();
    
    low_cmd_suber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    low_cmd_suber_->InitChannel(std::bind(&DDSBridge::LowCmdHandler, this, std::placeholders::_1), 1);

    std::cout << "DDS publisher and subscriber initialized." << std::endl;
}

DDSBridge::~DDSBridge() {}

void DDSBridge::LowCmdHandler(const void* msg)
{
    const unitree_go::msg::dds_::LowCmd_& cmd = *static_cast<const unitree_go::msg::dds_::LowCmd_*>(msg);

    // Mapping motor commands
    if (cmd.motor_cmd().size() > 0)
        controllerLeft_.setJointCommand(1, cmd.motor_cmd()[0].q(), cmd.motor_cmd()[0].dq(), cmd.motor_cmd()[0].tau(), cmd.motor_cmd()[0].kp(), cmd.motor_cmd()[0].kd());
    if (cmd.motor_cmd().size() > 1)
        controllerLeft_.setJointCommand(2, cmd.motor_cmd()[1].q(), cmd.motor_cmd()[1].dq(), cmd.motor_cmd()[1].tau(), cmd.motor_cmd()[1].kp(), cmd.motor_cmd()[1].kd());
    if (cmd.motor_cmd().size() > 2)
        controllerRight_.setJointCommand(4, -cmd.motor_cmd()[2].q(), -cmd.motor_cmd()[2].dq(), -cmd.motor_cmd()[2].tau(), cmd.motor_cmd()[2].kp(), cmd.motor_cmd()[2].kd());
    if (cmd.motor_cmd().size() > 3)
        controllerRight_.setJointCommand(5, -cmd.motor_cmd()[3].q(), -cmd.motor_cmd()[3].dq(), -cmd.motor_cmd()[3].tau(), cmd.motor_cmd()[3].kp(), cmd.motor_cmd()[3].kd());
    if (cmd.motor_cmd().size() > 4)
        canController_.setCommand(2, -cmd.motor_cmd()[4].dq(), -cmd.motor_cmd()[4].tau(), cmd.motor_cmd()[4].kd());
    if (cmd.motor_cmd().size() > 5)
        canController_.setCommand(1, cmd.motor_cmd()[5].dq(), cmd.motor_cmd()[5].tau(), cmd.motor_cmd()[5].kd());
}

void DDSBridge::PublishLowState()
{
    unitree_go::msg::dds_::LowState_ msg;
    std::size_t n = msg.motor_state().size();
    if (n < 6) {
        std::cerr << "Error: msg.motor_state() size is " << n << ", expected at least 6!" << std::endl;
        return;
    }

    // mapping motor states
    if (controllerLeft_.getMotors().size() > 0) {
        const auto& m = controllerLeft_.getMotors()[0];
        msg.motor_state()[0].q() = m.getJointPosition();
        msg.motor_state()[0].dq() = m.getJointVelocity();
        msg.motor_state()[0].tau_est() = m.getJointTorque();
    }
    if (controllerLeft_.getMotors().size() > 1) {
        const auto& m = controllerLeft_.getMotors()[1];
        msg.motor_state()[1].q() = m.getJointPosition();
        msg.motor_state()[1].dq() = m.getJointVelocity();
        msg.motor_state()[1].tau_est() = m.getJointTorque();
    }
    if (controllerRight_.getMotors().size() > 0) {
        const auto& m = controllerRight_.getMotors()[0];
        msg.motor_state()[2].q() = -m.getJointPosition();
        msg.motor_state()[2].dq() = -m.getJointVelocity();
        msg.motor_state()[2].tau_est() = -m.getJointTorque();
    }
    if (controllerRight_.getMotors().size() > 1) {
        const auto& m = controllerRight_.getMotors()[1];
        msg.motor_state()[3].q() = -m.getJointPosition();
        msg.motor_state()[3].dq() = -m.getJointVelocity();
        msg.motor_state()[3].tau_est() = -m.getJointTorque();
    }
    // CAN motors
    if (canController_.getMotors().size() > 1) {
        const auto& m = canController_.getMotors()[1];
        msg.motor_state()[4].q() = -m.data.q;
        msg.motor_state()[4].dq() = -m.data.dq;
        msg.motor_state()[4].tau_est() = -m.data.tau;
    }
    if (canController_.getMotors().size() > 0) {
        const auto& m = canController_.getMotors()[0];
        msg.motor_state()[5].q() = m.data.q;
        msg.motor_state()[5].dq() = m.data.dq;
        msg.motor_state()[5].tau_est() = m.data.tau;
    }

    // --- IMU data assignment ---
    if (imuData_) {
        std::lock_guard<std::mutex> lock(imuData_->mtx);
        for (int i = 0; i < 4; ++i)
            msg.imu_state().quaternion()[i] = imuData_->quaternion[i];
        for (int i = 0; i < 3; ++i) {
            msg.imu_state().rpy()[i] = imuData_->rpy[i];
            msg.imu_state().gyroscope()[i] = imuData_->gyro[i];
            msg.imu_state().accelerometer()[i] = imuData_->accel[i];
        }
    }

    low_state_puber_->Write(msg);
}

void DDSBridge::Run()
{
    while (true) {
        PublishLowState();
        std::this_thread::sleep_for(std::chrono::milliseconds(2)); // 500Hz
    }
}