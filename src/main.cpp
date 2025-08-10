#include "main.h"
#include "system_identification.hpp"
#include <vector>
#include <cmath>

using namespace motor_characterization;

// Global variables for motor characterization
SystemIdentification motorSysId;
bool isCharacterizing = false;
std::vector<double> velocityHistory;
std::vector<double> timeHistory;
const int HISTORY_SIZE = 10;

// Motor to characterize (adjust port as needed)
pros::Motor characterizationMotor(1);

/**
 * @brief Calculate acceleration using finite differences
 */
double calculateAcceleration(const std::vector<double>& velocities, const std::vector<double>& times) {
    if (velocities.size() < 2 || times.size() < 2) return 0.0;
    
    size_t n = velocities.size();
    double dt = times[n-1] - times[n-2];
    if (dt < 1e-6) return 0.0;
    
    return (velocities[n-1] - velocities[n-2]) / dt;
}

/**
 * @brief Run complete motor characterization in 20 seconds
 */
void runMotorCharacterization() {
    if (isCharacterizing) {
        return;
    }
    
    isCharacterizing = true;
    
    // Clear previous data
    motorSysId.clearData();
    
    // Define voltage test points in millivolts with alternating pattern to test acceleration
    // Pattern: low -> high -> low -> high -> etc. to create acceleration/deceleration
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
    int sampleRate = 10; // 100Hz sampling
    
    pros::lcd::clear();
    pros::lcd::print(0, "Starting Characterization");
    pros::lcd::print(1, "20 seconds total");
    pros::lcd::print(2, "Testing acceleration");
    pros::delay(1000);
    
    // Collect data for each voltage level
    for (size_t i = 0; i < testVoltages.size(); ++i) {
        int voltage = testVoltages[i];
        
        // Update LCD with progress
        pros::lcd::clear();
        pros::lcd::print(0, "Characterizing...");
        pros::lcd::print(1, "Test %d/%d: %d mV", i + 1, totalVoltages, voltage);
        pros::lcd::print(2, "Time: %.1fs", timePerVoltage / 1000.0);
        
        // Clear history for this test
        velocityHistory.clear();
        timeHistory.clear();
        
        // Apply voltage to motor using move_voltage
        characterizationMotor.move_voltage(voltage);
        
        // Collect data for the allocated time
        uint32_t startTime = pros::millis();
        uint32_t lastSampleTime = startTime;
        
        while (pros::millis() - startTime < timePerVoltage) {
            uint32_t currentTime = pros::millis();
            
            if (currentTime - lastSampleTime >= sampleRate) {
                double timestamp = (currentTime - startTime) / 1000.0;
                double velocity = characterizationMotor.get_actual_velocity();
                
                velocityHistory.push_back(velocity);
                timeHistory.push_back(timestamp);
                
                // Keep only recent history
                if (velocityHistory.size() > HISTORY_SIZE) {
                    velocityHistory.erase(velocityHistory.begin());
                    timeHistory.erase(timeHistory.begin());
                }
                
                lastSampleTime = currentTime;
                
                // Calculate acceleration and add data point
                if (velocityHistory.size() >= 2) {
                    double acceleration = calculateAcceleration(velocityHistory, timeHistory);
                    // Convert voltage from mV to V for data storage
                    double voltageV = voltage / 1000.0;
                    motorSysId.addDataPoint(voltageV, velocity, acceleration, timestamp);
                }
            }
            
            pros::delay(5);
        }
        
        // Stop motor
        characterizationMotor.move_voltage(0);
    }
    
    // Perform system identification
    pros::lcd::clear();
    pros::lcd::print(0, "Analyzing Data...");
    pros::lcd::print(1, "Total Points: %zu", motorSysId.getDataPointCount());
    
    bool success = motorSysId.identify(true, true); // Include static friction and acceleration
    
    if (success) {
        // Export data if SD card is available
        motorSysId.exportToCSV("/usd/motor_characterization.csv");
    }
    
    isCharacterizing = false;
}

/**
 * @brief Display motor characteristics on LCD
 */
void displayMotorCharacteristics() {
    if (!motorSysId.isSystemIdentified()) {
        pros::lcd::clear();
        pros::lcd::print(0, "No Characterization Data");
        pros::lcd::print(1, "Press A to Start");
        return;
    }
    
    FeedforwardConstants constants = motorSysId.getConstants();
    
    pros::lcd::clear();
    pros::lcd::print(0, "Motor Characteristics");
    pros::lcd::print(1, "kS: %.2f kV: %.3f kA: %.4f", constants.kS, constants.kV, constants.kA);
    pros::lcd::print(2, "R^2: %.3f", motorSysId.getRSquared());
    pros::lcd::print(3, "Data Points: %zu", motorSysId.getDataPointCount());
    
    // Calculate max velocity estimate (using 12V max)
    double maxVoltage = 12.0;
    double maxVelocity = (maxVoltage - constants.kS) / constants.kV;
    pros::lcd::print(4, "Max Vel: %.0f RPM", maxVelocity);
    
    // Show voltage for 100 RPM
    double voltage100 = constants.calculate(100.0, 0.0);
    pros::lcd::print(5, "100RPM: %.1fV", voltage100);
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

    while (true) {
        // A button: Run characterization (every time it's pressed)
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A) && !isCharacterizing) {
            runMotorCharacterization();
        }
        
        // Display motor characteristics on LCD
        if (!isCharacterizing) {
            displayMotorCharacteristics();
        }
        
        pros::delay(100); // Update every 100ms
    }
}