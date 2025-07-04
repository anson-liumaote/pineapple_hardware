#include "MotorController.h"
#include "CanMotorController.h"
#include <thread>
#include <chrono>
#include <atomic>
#include <cmath>
#include <iostream>

int main() {
    // --- Serial Motor Controllers ---
    MotorController controllerLeft("/dev/ttyUSB2", {1, 2});
    MotorController controllerRight("/dev/ttyUSB1", {4, 5});

    // Enable readonly mode for safety
    controllerLeft.enableReadonlyMode();
    controllerRight.enableReadonlyMode();

    // --- CAN Motor Controller ---
    CanMotorController canController("/dev/USB2CAN0", {1, 2});

    // --- Start threads for serial controllers ---
    std::thread threadLeft(&MotorController::runLoop, &controllerLeft);
    std::thread threadRight(&MotorController::runLoop, &controllerRight);

    // --- Start CAN communication thread ---
    std::atomic<bool> running{true};
    std::thread can_thread([&](){
        canController.runLoop(running);
    });

    // Wait for initial data
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Initialize and set limits
    controllerLeft.initializeMotorAndSetLimits(1, 90.0, 30.0, 20.0, 7.0, -7.0, 10, 0.0);
    controllerLeft.initializeMotorAndSetLimits(2, 90.0, 30.0, 20.0, 7.0, -7.0, 10, 0.0);
    controllerRight.initializeMotorAndSetLimits(4, 90.0, 30.0, 20.0, 7.0, -7.0, 10, 0.0);
    controllerRight.initializeMotorAndSetLimits(5, 90.0, 30.0, 20.0, 7.0, -7.0, 10, 0.0);

    // --- Control loop parameters ---
    double period = 6.0;
    double amplitude = 1.57; // For serial (position, rad)
    double can_amplitude = 10.0; // For CAN (velocity, rad/s)
    auto t_start = std::chrono::steady_clock::now();

    for (int i = 0; i < 1000; ++i) {
        auto t_now = std::chrono::steady_clock::now();
        double time = std::chrono::duration<double>(t_now - t_start).count();

        // --- Serial: Generate sinusoidal position command ---
        double offset = amplitude * std::sin(2.0 * M_PI * time / period);
        double target_pos0 = offset;
        double target_pos4 = -offset;
        double target_posB0 = offset;

        // Position control for serial motors
        controllerLeft.setJointCommand(1, target_pos0, 0.0, 0.0, 0.0, 0.0);
        controllerLeft.setJointCommand(2, target_pos4, 0.0, 0.0, 0.0, 0.0);
        controllerRight.setJointCommand(4, target_posB0, 0.0, 0.0, 0.0, 0.0);
        controllerRight.setJointCommand(5, target_posB0, 0.0, 0.0, 0.0, 0.0);

        // --- CAN: Sine wave dq command for motor 1, zero for motor 2 ---
        canController.setCommand(1, can_amplitude * std::sin(2.0 * M_PI * time / period), 0.0, 0.0);
        canController.setCommand(2, 0.0, 0.0, 0.0);

        // --- Print serial motor data ---
        for (const auto& motor : controllerLeft.getMotors()) {
            std::cout << "[Serial A] Motor " << motor.id
                      << " data: q=" << motor.data.q
                      << ", dq=" << motor.data.dq
                      << ", tau=" << motor.data.tau
                      << ", temp=" << motor.data.temp << std::endl;
        }
        for (const auto& motor : controllerRight.getMotors()) {
            std::cout << "[Serial B] Motor " << motor.id
                      << " data: q=" << motor.data.q
                      << ", dq=" << motor.data.dq
                      << ", tau=" << motor.data.tau
                      << ", temp=" << motor.data.temp << std::endl;
        }

        // --- Print CAN motor data ---
        // for (const auto& motor : canController.getMotors()) {
        //     std::cout << "[CAN] Motor " << motor.id
        //               << " data: q=" << motor.data.q
        //               << ", dq=" << motor.data.dq
        //               << ", tau=" << motor.data.tau
        //               << ", temp=" << motor.data.temp << std::endl;
        // }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    running = false;
    can_thread.join();
    threadLeft.join();
    threadRight.join();
    return 0;
}