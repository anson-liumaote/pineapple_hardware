#include "DDS_bridge.h"
#include "ImuSharedData.h"
#include <thread>
#include <atomic>
#include <unitree/robot/channel/channel_factory.hpp> // Add this include

void imuRS485Thread(std::atomic<bool>& running, ImuSharedData* imuData, const char* portname);

int main() {
    // --- DDS/Unitree ChannelFactory initialization ---
    ChannelFactory::Instance()->Init(1, "lo");

    // --- Start CAN communication thread ---
    std::atomic<bool> running{true};

    // --- IMU shared data and thread ---
    ImuSharedData imuData1, imuData2, imuData3, imuData4, imuData5;
    ImuSharedData* imuDataArr[5] = {&imuData1, &imuData2, &imuData3, &imuData4, &imuData5};
    std::thread imu_thread1(imuRS485Thread, std::ref(running), &imuData1, "/dev/ttyUSB0");
    std::thread imu_thread2(imuRS485Thread, std::ref(running), &imuData2, "/dev/ttyUSB1");
    std::thread imu_thread3(imuRS485Thread, std::ref(running), &imuData3, "/dev/ttyUSB2");
    std::thread imu_thread4(imuRS485Thread, std::ref(running), &imuData4, "/dev/ttyUSB3");

    // Wait for initial data
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "[DDS_Bridge] initialization done, waiting for data..." << std::endl;

    // --- DDS Bridge ---
    DDSBridge bridge(imuDataArr);

    // Keep threads running until you want to stop
    std::cout << "[DDS_Bridge] Press Enter to stop..." << std::endl;
    std::cin.get();

    running = false;
    
    // Clean up threads
    imu_thread1.join();
    imu_thread2.join();
    imu_thread3.join();
    imu_thread4.join();
    return 0;
}