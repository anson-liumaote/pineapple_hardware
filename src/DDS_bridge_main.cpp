#include "DDS_bridge.h"
#include "ImuSharedData.h"
#include <thread>
#include <atomic>
#include <unitree/robot/channel/channel_factory.hpp> // Add this include

void imuThreadFunc(std::atomic<bool>& running, ImuSharedData* imuData); // Forward declaration

int main(int argc, const char **argv) {
    // --- DDS/Unitree ChannelFactory initialization ---
    if (argc < 2)
    {
        ChannelFactory::Instance()->Init(1, "lo");
    }
    else
    {
        ChannelFactory::Instance()->Init(1, argv[1]);
    }
    // ChannelFactory::Instance()->Init(1, "lo"); // domain_id=0, interface="lo"

    // --- Serial Motor Controllers ---
    MotorController controllerHip("/dev/rs485-1", {0, 1});
    MotorController controllerThigh("/dev/rs485-2", {0, 1});
    MotorController controllerCalf("/dev/rs485-a1", {0, 1}, MotorType::A1);
    // controllerHip.enableReadonlyMode();
    // controllerThigh.enableReadonlyMode();
    // controllerCalf.enableReadonlyMode();

    // --- CAN Motor Controller ---
    DMCanMotorController canController("/dev/ttyACM0", {1, 2});

    // --- Start threads for serial controllers ---
    std::thread threadHip(&MotorController::runLoop, &controllerHip);
    std::thread threadThigh(&MotorController::runLoop, &controllerThigh);
    std::thread threadCalf(&MotorController::runLoop, &controllerCalf);

    // --- Start CAN communication thread ---
    std::atomic<bool> running{true};
    // std::thread can_thread([&](){
    //     canController.runLoop(running);
    // });

    // --- IMU shared data and thread ---
    ImuSharedData imuData;
    std::thread imu_thread([&](){
        imuThreadFunc(running, &imuData);
    });

    // Wait for initial data
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Initialize and set limits
    // controllerLeft.initializeMotorAndSetLimits(1, 90.0, 30.0, 20.0, 2.05, -1.2, 10, -1.48);  
    // controllerLeft.initializeMotorAndSetLimits(2, 90.0, 30.0, 20.0, 0.73, -3.25, 10, 3.2);
    // controllerRight.initializeMotorAndSetLimits(4, 90.0, 30.0, 20.0, 1.2, -2.05, 10, 1.48);  // set limit and offset inverse
    // controllerRight.initializeMotorAndSetLimits(5, 90.0, 30.0, 20.0, 3.25, -0.73, 10, -3.2);  // set limit and offset inverse
    controllerHip.initializeMotorAndSetLimits(0, 90.0, 30.0, 20.0, 0.52358, -0.17444, 10, 0);   // set limit and offset inverse
    controllerHip.initializeMotorAndSetLimits(1, 90.0, 30.0, 20.0, 0.17444, -0.52358, 10, 0);   // set limit and offset inverse
    controllerThigh.initializeMotorAndSetLimits(0, 90.0, 30.0, 20.0, 1.571, 0.0, 10, -1.49);
    controllerThigh.initializeMotorAndSetLimits(1, 90.0, 30.0, 20.0, 0.0, -1.571, 10, 1.49);    // set limit and offset inverse
    controllerCalf.initializeMotorAndSetLimits(0, 90.0, 21.0, 33.5, 0.0, -3.3, 10, 3.14);
    controllerCalf.initializeMotorAndSetLimits(1, 90.0, 21.0, 33.5, 3.3, 0.0, 10, -3.14);   // set limit and offset inverse
    std::cout << "[DDS_Bridge] Initilization done, waiting for data..." << std::endl;
    
    // --- DDS Bridge ---
    DDSBridge bridge(controllerHip, controllerThigh, controllerCalf, canController, &imuData);
    // Keep threads running until you want to stop
    std::cout << "[DDS_Bridge] Press Ctrl+C to stop..." << std::endl;
    std::cin.get();

    running = false;
    imu_thread.join();
    // can_thread.join();
    threadHip.join();
    threadThigh.join();
    threadCalf.join();
    return 0;
}