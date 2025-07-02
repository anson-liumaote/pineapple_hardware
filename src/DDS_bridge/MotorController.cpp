#include "MotorController.h"
#include <unistd.h>
#include <chrono>
#include <iostream>
#include <cmath>

Motor::Motor(int motor_id, MotorType type) : id(motor_id), motor_type(type) {
    cmd.motorType = motor_type;
    data.motorType = motor_type;
    cmd.mode = queryMotorMode(motor_type, MotorMode::FOC);
    cmd.id = id;
    cmd.kp = 0.0;
    cmd.kd = 0.0;
    cmd.q = 0.0;
    cmd.dq = 0.0;
    cmd.tau = 0.0;
    
    // Set gear ratio based on motor type
    gear_ratio = GearRatio(queryGearRatio(motor_type));
    
    std::cout << "Motor " << id << " initialized with type " << (int)motor_type 
              << " and gear ratio " << gear_ratio.ratio << std::endl;
}

void Motor::setMotorType(MotorType type) {
    motor_type = type;
    cmd.motorType = type;
    data.motorType = type;
    cmd.mode = queryMotorMode(type, MotorMode::FOC);
    
    // Update gear ratio based on new motor type
    gear_ratio = GearRatio(queryGearRatio(type));
    
    std::cout << "Motor " << id << " type changed to " << (int)type 
              << " with gear ratio " << gear_ratio.ratio << std::endl;
}

void Motor::setGearRatio(double ratio) {
    gear_ratio = GearRatio(ratio);
    std::cout << "Motor " << id << " gear ratio manually set to: " << ratio << std::endl;
}

bool Motor::checkLimits() {
    bool limit_exceeded = false;
    
    // Check temperature
    if (data.temp > limits.max_temp) {
        std::cout << "WARNING: Motor " << id << " temperature exceeded: " 
                  << data.temp << "°C > " << limits.max_temp << "°C" << std::endl;
        limit_exceeded = true;
    }
    
    // Check velocity (using gear ratio for proper scaling)
    double joint_velocity = getJointVelocity();
    if (std::abs(joint_velocity) > limits.max_velocity) {
        std::cout << "WARNING: Motor " << id << " velocity exceeded: " 
                  << std::abs(joint_velocity) << " > " << limits.max_velocity << " rad/s" << std::endl;
        limit_exceeded = true;
    }
    
    // Check position limits (using gear ratio for proper scaling)
    double joint_position = getJointPosition(); // Get joint position with offset
    if (joint_position > limits.max_position || joint_position < limits.min_position) {
        std::cout << "WARNING: Motor " << id << " position limit exceeded: " 
                  << joint_position << " rad" << std::endl;
        limit_exceeded = true;
    }
    
    // Check torque limits (using gear ratio for proper scaling)
    double joint_torque = getJointTorque();;
    if (std::abs(joint_torque) > limits.max_torque) {
        std::cout << "WARNING: Motor " << id << " torque exceeded: " 
                  << std::abs(joint_torque) << " > " << limits.max_torque << " Nm" << std::endl;
        limit_exceeded = true;
    }
    
    // Check for motor errors
    if (data.merror != 0) {
        std::cout << "WARNING: Motor " << id << " error code: " << data.merror << std::endl;
        error_count++;
        if (error_count > limits.max_error_count) {
            std::cout << "CRITICAL: Motor " << id << " error count exceeded!" << std::endl;
            limit_exceeded = true;
        }
    } else {
        error_count = 0; // Reset error count if no error
    }
    
    return limit_exceeded;
}

void Motor::enableProtection() {
    protection_active = true;
    std::cout << "Protection ENABLED for Motor " << id << std::endl;
}

void Motor::disableProtection() {
    protection_active = false;
    std::cout << "Protection DISABLED for Motor " << id << std::endl;
}

void Motor::enableReadonlyMode() {
    readonly_mode = true;
    std::cout << "Readonly mode ENABLED for Motor " << id << std::endl;
}

void Motor::disableReadonlyMode() {
    readonly_mode = false;
    std::cout << "Readonly mode DISABLED for Motor " << id << std::endl;
}

void Motor::update(SerialPort& serial) {
    // If in readonly mode, set all control parameters to 0 and skip protection
    if (readonly_mode) {
        cmd.kp = 0.0;
        cmd.kd = 0.0;
        // cmd.q = 0.0;
        cmd.dq = 0.0;
        cmd.tau = 0.0;
        
        // Send command and receive data (readonly - no protection detection)
        serial.sendRecv(&cmd, &data);
        return;
    }
    
    // Normal operation: Check safety limits with current data (from previous cycle)
    if (checkLimits()) {
        if (!protection_active) {
            enableProtection();
        }
    }
    
    // Apply protection before sending commands
    if (protection_active) {
        // Set motor to safe state
        cmd.kp = 0.0;
        cmd.kd = 0.025;
        cmd.tau = 0.0;
        cmd.dq = 0.0;
        
        // Position protection: if near limits, command safe position (using gear ratio)
        // double joint_position = getJointPosition(); // Get joint position with offset
        // if (joint_position > limits.max_position * 0.9) {
        //     setJointPosition(limits.max_position * 0.8); // Move to 80% of max
        //     std::cout << "Motor " << id << " position protection: commanding safe position" << std::endl;
        // } else if (joint_position < limits.min_position * 0.9) {
        //     setJointPosition(limits.min_position * 0.8); // Move to 80% of min
        //     std::cout << "Motor " << id << " position protection: commanding safe position" << std::endl;
        // }
    } else {
        // Normal operation: apply command limits
        // Clamp torque command to safe limits (convert joint torque limit to motor torque)
        double max_motor_torque = gear_ratio.toMotorTorque(limits.max_torque);
        if (std::abs(cmd.tau) > max_motor_torque) {
            cmd.tau = (cmd.tau > 0) ? max_motor_torque : -max_motor_torque;
            std::cout << "Motor " << id << " torque clamped to ±" << limits.max_torque << " Nm (joint)" << std::endl;
        }
        
        // Clamp velocity command to safe limits (using gear ratio)
        double joint_vel_cmd = gear_ratio.toJointUnits(cmd.dq);
        if (std::abs(joint_vel_cmd) > limits.max_velocity) {
            double clamped_joint_vel = (joint_vel_cmd > 0) ? limits.max_velocity : -limits.max_velocity;
            cmd.dq = gear_ratio.toMotorUnits(clamped_joint_vel);
            std::cout << "Motor " << id << " velocity clamped to ±" << limits.max_velocity << " rad/s" << std::endl;
        }
        
        // Position protection: prevent commanding positions beyond limits (using gear ratio)
        double joint_pos_cmd = gear_ratio.toJointUnits(cmd.q) - joint_offset;
        if (joint_pos_cmd > limits.max_position) {
            // setJointPosition(limits.max_position);
            std::cout << "Motor " << id << " position command clamped to max limit" << std::endl;
        } else if (joint_pos_cmd < limits.min_position) {
            // setJointPosition(limits.min_position);
            std::cout << "Motor " << id << " position command clamped to min limit" << std::endl;
        }
    }
    
    // Send command and receive data
    serial.sendRecv(&cmd, &data);
    double error = data.q - cmd.q; // Calculate error in motor units
    if (std::abs(error) < 0.01) {
        std::cout << "Motor " << id << " command executed successfully." << std::endl;
    }
}

MotorController::MotorController(const std::string& port, const std::vector<int>& motor_ids, MotorType type)
    : serial(port)
{
    for (int id : motor_ids) {
        motors.emplace_back(id, type);
    }
}

void MotorController::enableReadonlyMode() {
    for (auto& motor : motors) {
        motor.enableReadonlyMode();
    }
    std::cout << "Readonly mode ENABLED for all motors" << std::endl;
}

void MotorController::disableReadonlyMode() {
    for (auto& motor : motors) {
        motor.disableReadonlyMode();
    }
    std::cout << "Readonly mode DISABLED for all motors" << std::endl;
}

void MotorController::runLoop() {
    while (true) {
        auto loop_start = std::chrono::steady_clock::now();

        for (auto& motor : motors) {
            motor.update(serial);
        }

        auto loop_end = std::chrono::steady_clock::now();
        double loop_time = std::chrono::duration<double>(loop_end - loop_start).count();
        double freq = (loop_time > 0.0) ? 1.0 / loop_time : 0.0;

        // std::cout << "[Controller Thread] Frequency: " << freq << " Hz" << std::endl;
    }
}

void MotorController::printMotorData() {
    for (const auto& motor : motors) {
        std::cout << std::endl;
        std::cout << "Motor ID: " << motor.id << std::endl;
        std::cout << "Gear Ratio: " << motor.gear_ratio.ratio << std::endl;
        std::cout << "motor.q: " << motor.data.q << " (joint: " << motor.gear_ratio.toJointUnits(motor.data.q) << ")" << std::endl;
        std::cout << "motor.temp: " << motor.data.temp << std::endl;
        std::cout << "motor.dq: " << motor.data.dq << " (joint: " << motor.gear_ratio.toJointUnits(motor.data.dq) << ")" << std::endl;
        std::cout << "motor.merror: " << motor.data.merror << std::endl;
        std::cout << "Protection: " << (motor.protection_active ? "ACTIVE" : "INACTIVE") << std::endl;
        std::cout << "Readonly Mode: " << (motor.readonly_mode ? "ENABLED" : "DISABLED") << std::endl;
        std::cout << "Error count: " << motor.error_count << std::endl;
        std::cout << std::endl;
    }
}

Motor* MotorController::getMotor(int motor_id) {
    for (auto& motor : motors) {
        if (motor.id == motor_id) {
            return &motor;
        }
    }
    return nullptr;
}

double MotorController::getJointPosition(int motor_id) {
    Motor* motor = getMotor(motor_id);
    return motor ? motor->getJointPosition() : 0.0;
}

double MotorController::getJointVelocity(int motor_id) {
    Motor* motor = getMotor(motor_id);
    return motor ? motor->getJointVelocity() : 0.0;
}

double MotorController::getJointTorque(int motor_id) {
    Motor* motor = getMotor(motor_id);
    return motor ? motor->getJointTorque() : 0.0;
}

void MotorController::setJointPosition(int motor_id, double joint_pos) {
    Motor* motor = getMotor(motor_id);
    if (motor && !motor->readonly_mode) {
        motor->setJointPosition(joint_pos);
    }
}

void MotorController::setJointVelocity(int motor_id, double joint_vel) {
    Motor* motor = getMotor(motor_id);
    if (motor && !motor->readonly_mode) {
        motor->setJointVelocity(joint_vel);
    }
}

void MotorController::setJointTorque(int motor_id, double joint_torque) {
    Motor* motor = getMotor(motor_id);
    if (motor && !motor->readonly_mode) {
        motor->setJointTorque(joint_torque);
    }
}

void MotorController::setGains(int motor_id, double kp, double kd) {
    Motor* motor = getMotor(motor_id);
    if (motor && !motor->readonly_mode) {
        motor->setGains(kp, kd);
    }
}

void MotorController::setJointCommand(int motor_id, double pos, double vel, double torque, double kp, double kd) {
    Motor* motor = getMotor(motor_id);
    if (motor && !motor->readonly_mode) {
        motor->setJointCommand(pos, vel, torque, kp, kd);
    }
}
