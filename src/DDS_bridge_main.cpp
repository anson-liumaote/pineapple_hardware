#include "DDS_bridge.h"
#include "ImuSharedData.h"
#include <thread>
#include <atomic>
#include <unitree/robot/channel/channel_factory.hpp> // Add this include

// void imuCANThread(std::atomic<bool>& running, ImuSharedData* imuData); // Forward declaration
void imuRS485Thread(std::atomic<bool>& running, ImuSharedData* imuData, const char* portname);

int main() {
    // --- DDS/Unitree ChannelFactory initialization ---
    ChannelFactory::Instance()->Init(1, "lo"); // domain_id=0, interface="lo"

    // --- Serial Motor Controllers ---
    // MotorController controllerLeft("/dev/ttyUSB2", {1, 2});
    // MotorController controllerRight("/dev/ttyUSB1", {4, 5});
    // controllerLeft.enableReadonlyMode();
    // controllerRight.enableReadonlyMode();

    // --- CAN Motor Controller ---
    // CanMotorController canController("/dev/USB2CAN0", {1, 2});

    // --- Start threads for serial controllers ---
    // std::thread threadLeft(&MotorController::runLoop, &controllerLeft);
    // std::thread threadRight(&MotorController::runLoop, &controllerRight);

    // --- Start CAN communication thread ---
    std::atomic<bool> running{true};
    // std::thread can_thread([&](){
    //     canController.runLoop(running);
    // });

    // --- IMU shared data and thread ---
    ImuSharedData imuData1, imuData2, imuData3, imuData4, imuData5;
    ImuSharedData* imuDataArr[5] = {&imuData1, &imuData2, &imuData3, &imuData4, &imuData5};
    std::thread imu_thread1(imuRS485Thread, std::ref(running), &imuData1, "/dev/ttyUSB0");
    // std::thread imu_thread2(imuRS485Thread, std::ref(running), &imuData2, "/dev/ttyUSB1");
    // 依需求新增 thread3~5

    // Wait for initial data
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Initialize and set limits
    // controllerLeft.initializeMotorAndSetLimits(1, 90.0, 30.0, 20.0, 2.05, -1.2, 10, -1.48);  
    // controllerLeft.initializeMotorAndSetLimits(2, 90.0, 30.0, 20.0, 0.73, -3.25, 10, 3.2);
    // controllerRight.initializeMotorAndSetLimits(4, 90.0, 30.0, 20.0, 1.2, -2.05, 10, 1.48);  // set limit and offset inverse
    // controllerRight.initializeMotorAndSetLimits(5, 90.0, 30.0, 20.0, 3.25, -0.73, 10, -3.2);  // set limit and offset inverse
    std::cout << "initilization done, waiting for data..." << std::endl;
    
    // --- DDS Bridge ---
    DDSBridge bridge(imuDataArr);
    bridge.Run();

    running = false;
    imu_thread1.join();
    // imu_thread2.join();
    // can_thread.join();
    // threadLeft.join();
    // threadRight.join();
    return 0;
}