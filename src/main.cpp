#include "main.h"
#include "system_identification.hpp"
#include <vector>
#include <cmath>
#include <iostream>

using namespace motor_characterization;

// Motor to characterize (adjust port as needed)
pros::Motor characterizationMotor(1);

/**
 * @brief Run complete motor characterization in 20 seconds
 */
void runMotorCharacterization() {
    // Local variables instead of globals
    double previousVelocity = 0.0;
    double previousTime = 0.0;
    
    // Create system identification object locally
    SystemIdentification motorSysId;
    
    // Define voltage test points in millivolts with alternating pattern to test acceleration
    std::vector<int> testVoltages = {
        0,      // Start at zero
        6000,   // Jump to high positive
        0,      // Back to zero (deceleration)
        -6000,  // Jump to high negative
        0,      // Back to zero (deceleration)
        12000,  // Jump to max positive
        0,      // Back to zero (deceleration)
        -12000, // Jump to max negative
        0,      // Back to zero (deceleration)
        3000,   // Medium positive
        0,      // Back to zero
        -3000   // Medium negative
    };
    
    // Calculate time per voltage level (20 seconds total)
    int totalVoltages = testVoltages.size();
    int timePerVoltage = 20000 / totalVoltages; // 20 seconds / number of voltages
    
    // Simple LCD display - just show we're starting
    pros::lcd::clear();
    pros::lcd::print(0, "Starting Characterization");
    pros::lcd::print(1, "20 seconds total");
    
    // Collect data for each voltage level
    for (size_t i = 0; i < testVoltages.size(); ++i) {
        int voltage = testVoltages[i];
        
        // Simple LCD display - just show progress
        pros::lcd::print(0, "Test %d/%d", i + 1, totalVoltages);
        
        // Reset previous values for new voltage level
        previousVelocity = 0.0;
        previousTime = 0.0;
        
        // Apply voltage to motor using move_voltage
        characterizationMotor.move_voltage(voltage);
        
        // Simplified data collection loop
        uint32_t startTime = pros::millis();
        
        while (pros::millis() - startTime < timePerVoltage) {
            double currentTime = (pros::millis() - startTime) / 1000.0;
            double currentVelocity = characterizationMotor.get_actual_velocity();
            
            // Direct acceleration calculation without history arrays
            if (previousTime > 0) {
                double dt = currentTime - previousTime;
                if (dt > 0.001) { // Avoid division by zero
                    double acceleration = (currentVelocity - previousVelocity) / dt;
                    // Convert voltage from mV to V for data storage
                    double voltageV = voltage / 1000.0;
                    motorSysId.addDataPoint(voltageV, currentVelocity, acceleration, currentTime);
                }
            }
            
            previousVelocity = currentVelocity;
            previousTime = currentTime;
            pros::delay(10); // 100Hz sampling
        }
        
        // Stop motor
        characterizationMotor.move_voltage(0);
    }
    
    // Perform system identification
    pros::lcd::clear();
    pros::lcd::print(0, "Analyzing Data...");
    pros::lcd::print(1, "Total Points: %zu", motorSysId.getDataPointCount());
    
    bool success = motorSysId.identify(true, true); // Include static friction and acceleration
    
    // No CSV export - removed for simplicity
}

/**
 * @brief Display motor characteristics on LCD
 */
void displayMotorCharacteristics() {
    // Create local system identification object
    SystemIdentification motorSysId;
    
    if (!motorSysId.isSystemIdentified()) {
        // pros::lcd::clear();
        pros::lcd::print(0, "No Characterization Data");
        pros::lcd::print(1, "Press A to Start");
        return;
    }
    
    FeedforwardConstants constants = motorSysId.getConstants();
    
    pros::lcd::print(0, "Motor Characteristics");
    pros::lcd::print(1, "kS: %.2f kV: %.3f kA: %.4f", constants.kS, constants.kV, constants.kA);
    pros::lcd::print(2, "R^2: %.3f", motorSysId.getRSquared());
    pros::lcd::print(3, "Data Points: %zu", motorSysId.getDataPointCount());
    
    // Calculate max velocity estimate (using 12V max)
    double maxVoltage = 12.0;
    double maxVelocity = (maxVoltage - constants.kS) / constants.kV;
    // pros::lcd::print(4, "Max Vel: %.0f RPM", maxVelocity);
    
    // Show voltage for 100 RPM
    double voltage100 = constants.calculate(100.0, 0.0);
    // pros::lcd::print(5, "100RPM: %.1fV", voltage100);
}

/**
 * A callback function for LLEMU's center button.
 */
void on_center_button() {
    // This can be used for additional functionality if needed
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 */
void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(0, "Motor Characterization");
    pros::lcd::set_text(1, "Press A to Start");
    
    pros::lcd::register_btn1_cb(on_center_button);

    std::cout << "Initializing" << std::endl;
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 */
void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    bool isCharacterizing = false;

    while (true) {
        // A button: Run characterization (every time it's pressed)
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A) && !isCharacterizing) {
            isCharacterizing = true;
            runMotorCharacterization();
            isCharacterizing = false;
        }
        
        // Display motor characteristics on LCD
        if (!isCharacterizing) {
            displayMotorCharacteristics();
        }
        
        pros::delay(10); // Update every 100ms
    }
}