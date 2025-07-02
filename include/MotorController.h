#pragma once

#include <vector>
#include <string>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

class Motor {
public:
    // Gear ratio type for motor calculations
    struct GearRatio {
        double ratio;
        
        GearRatio() : ratio(1.0) {}
        GearRatio(double r) : ratio(r) {}
        
        // Helper methods for conversions
        double toMotorUnits(double joint_value) const { return joint_value * ratio; }
        double toJointUnits(double motor_value) const { return motor_value / ratio; }
        
        // Torque conversions (torque relationship is inverse of position/velocity)
        double toMotorTorque(double joint_torque) const { return joint_torque / ratio; }
        double toJointTorque(double motor_torque) const { return motor_torque * ratio; }
    };

    int id;
    MotorCmd cmd;
    MotorData data;
    MotorType motor_type;      // Motor type variable
    GearRatio gear_ratio;      // Gear ratio for this motor
    
    // Safety limits
    struct Limits {
        double max_temp = 80.0;         // Maximum temperature in Celsius
        double max_velocity = 30.0;     // Maximum velocity in rad/s
        double max_torque = 12.0;       // Maximum torque in Nm
        double max_position = 6.28;     // Maximum position in rad (1 revolution)
        double min_position = -6.28;    // Minimum position in rad
        int max_error_count = 5;        // Maximum consecutive errors before shutdown
    } limits;
    
    int error_count = 0;                // Track consecutive errors
    bool protection_active = false;     // Protection state
    bool readonly_mode = false;         // Readonly mode flag

    double joint_offset = 0.0; // Offset for this motor in joint space

    Motor(int motor_id, MotorType type = MotorType::GO_M8010_6);
    void update(SerialPort& serial);
    bool checkLimits();
    void enableProtection();
    void disableProtection();
    void enableReadonlyMode();
    void disableReadonlyMode();
    void setMotorType(MotorType type);
    void setGearRatio(double ratio);
    
    // Joint space data access
    double getJointPosition() const { return gear_ratio.toJointUnits(data.q) - joint_offset; }
    double getJointVelocity() const { return gear_ratio.toJointUnits(data.dq); }
    double getJointTorque() const { return gear_ratio.toJointTorque(data.tau); }  // Convert motor torque to joint torque
    
    // Joint space command methods
    void setJointPosition(double joint_pos) { cmd.q = gear_ratio.toMotorUnits(joint_pos + joint_offset); }
    void setJointVelocity(double joint_vel) { cmd.dq = gear_ratio.toMotorUnits(joint_vel); }
    void setJointTorque(double joint_torque) { cmd.tau = gear_ratio.toMotorTorque(joint_torque); }
    void setGains(double kp, double kd) {
        double ratio2 = gear_ratio.ratio * gear_ratio.ratio;
        cmd.kp = kp / ratio2;
        cmd.kd = kd / ratio2;
    }

    // Set joint offset
    void setJointOffset(double offset) { joint_offset = offset; }

    // Combined command method
    void setJointCommand(double pos, double vel, double torque, double kp, double kd) {
        setJointPosition(pos);
        setJointVelocity(vel);
        setJointTorque(torque);
        setGains(kp, kd);
    }

    // Set safety limits from outside (e.g., test.cpp)
    // The max_position and min_position are in joint space and will be shifted by joint_offset internally
    void setLimits(double max_temp, double max_velocity, double max_torque,
                   double max_position, double min_position, int max_error_count) {
        limits.max_temp = max_temp;
        limits.max_velocity = max_velocity;
        limits.max_torque = max_torque;
        limits.max_position = max_position;
        limits.min_position = min_position;
        limits.max_error_count = max_error_count;
        setJointPosition((max_position + min_position) / 2.0); // Set initial position to middle of limits to avoid immediate limit violation
    }

    // Initialize motor: set current joint position as offset plus urdf_offset and set limits
    void initializeAndSetLimits(double max_temp, double max_velocity, double max_torque,
                                double max_position, double min_position, int max_error_count,
                                double urdf_offset) {
        // Read current joint position as offset and add urdf_offset
        joint_offset = gear_ratio.toJointUnits(data.q) + urdf_offset;
        // Set safety limits (limits are shifted by offset inside setLimits)
        setLimits(max_temp, max_velocity, max_torque, max_position, min_position, max_error_count);
    }
};

class MotorController {
private:
    SerialPort serial;
    std::vector<Motor> motors;
    
    Motor* getMotor(int motor_id);

public:
    MotorController(const std::string& port, const std::vector<int>& motor_ids, MotorType type = MotorType::GO_M8010_6);
    void runLoop();
    void printMotorData();
    void enableReadonlyMode();
    void disableReadonlyMode();
    
    // Joint space data access by motor ID
    double getJointPosition(int motor_id);
    double getJointVelocity(int motor_id);
    double getJointTorque(int motor_id);
    
    // Joint space command methods by motor ID
    void setJointPosition(int motor_id, double joint_pos);
    void setJointVelocity(int motor_id, double joint_vel);
    void setJointTorque(int motor_id, double joint_torque);
    void setGains(int motor_id, double kp, double kd);
    
    // Combined command method by motor ID
    void setJointCommand(int motor_id, double pos, double vel, double torque, double kp, double kd);

    // Replace setMotorLimits with a more general function to initialize and set limits for a motor by ID
    void initializeMotorAndSetLimits(int motor_id, double max_temp, double max_velocity, double max_torque,
                                     double max_position, double min_position, int max_error_count,
                                     double urdf_offset) {
        Motor* m = getMotor(motor_id);
        if (m) m->initializeAndSetLimits(max_temp, max_velocity, max_torque, max_position, min_position, max_error_count, urdf_offset);
    }

    // Print joint position and offset for a given motor ID
    void printJointPositionAndOffset(int motor_id) {
        for (const auto& motor : motors) {
            if (motor.id == motor_id) {
                std::cout << "Motor " << motor_id << " - Pos: " << motor.getJointPosition()
                          << " (offset: " << motor.joint_offset << ")" << std::endl;
                return;
            }
        }
        std::cout << "Motor " << motor_id << " not found." << std::endl;
    }

    const std::vector<Motor>& getMotors() const { return motors; }
    std::vector<Motor>& getMotors() { return motors; }
};
