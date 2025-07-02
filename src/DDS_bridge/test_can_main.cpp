#include "CanMotorController.h"
#include <thread>
#include <chrono>
#include <atomic>
#include <cmath>
#include <iostream>

int main() {
    // Create controller and motors (IDs 1 and 2)
    CanMotorController controller("/dev/USB2CAN0", {1, 2});

    std::atomic<bool> running{true};
    std::thread can_thread([&](){
        controller.runLoop(running);
    });

    // Wait for initial data
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    double period = 6.0;
    double amplitude = 10.0;
    auto t_start = std::chrono::steady_clock::now();

    for (int i = 0; i < 1000; ++i) {
        auto t_now = std::chrono::steady_clock::now();
        double time = std::chrono::duration<double>(t_now - t_start).count();

        // Sine wave dq command for motor 1, zero for motor 2
        controller.setCommand(1, amplitude * std::sin(2.0 * M_PI * time / period), 0.0, 0.0);
        controller.setCommand(2, 0.0, 0.0, 0.0);

        // Print motor data
        for (const auto& motor : controller.getMotors()) {
            std::cout << "Motor " << motor.id
                      << " data: q=" << motor.data.q
                      << ", dq=" << motor.data.dq
                      << ", tau=" << motor.data.tau
                      << ", temp=" << motor.data.temp << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    running = false;
    can_thread.join();
    return 0;
}