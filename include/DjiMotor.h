#pragma once

#include <cstdint>

class DjiMotor {
public:
    struct Cmd {
        double dq = 0.0;     // Target velocity (rad/s)
        double tau = 0.0;    // Target torque (Nm)
        double kd = 0.0;     // Velocity gain
    };

    struct Data {
        double q = 0.0;      // Measured position (rad)
        double dq = 0.0;     // Measured velocity (rad/s)
        double tau = 0.0;    // Measured current/torque (A or Nm)
        double temp = 0.0;   // Temperature (degC)
    };

    int id = 0;
    double gear_ratio = 15.76; // Default gear ratio set to 15.76
    Cmd cmd;
    Data data;

    explicit DjiMotor(int motor_id)
        : id(motor_id) {}
};