#include "system_identification.hpp"
#include "main.h"
#include <vector>
#include <cmath>

using namespace motor_characterization;

// Global variables for data collection
SystemIdentification sysId;
bool isCollectingData = false;
std::vector<double> velocityHistory;
std::vector<double> timeHistory;
const int HISTORY_SIZE = 10; // Number of samples to use for acceleration calculation

/**
 * @brief Calculate acceleration using finite differences
 * @param velocities Vector of velocity measurements
 * @param times Vector of corresponding timestamps
 * @return Acceleration in RPM/s
 */
double calculateAcceleration(const std::vector<double>& velocities, const std::vector<double>& times) {
    if (velocities.size() < 2 || times.size() < 2) return 0.0;
    
    // Use the last two points for simple finite difference
    size_t n = velocities.size();
    double dt = times[n-1] - times[n-2];
    if (dt < 1e-6) return 0.0;
    
    return (velocities[n-1] - velocities[n-2]) / dt;
}

/**
 * @brief Collect motor data for system identification
 * @param motor The motor to collect data from
 * @param voltage The voltage to apply to the motor
 * @param duration_ms How long to collect data in milliseconds
 * @param sampleRate_ms Sample rate in milliseconds
 */
void collectMotorData(pros::Motor& motor, double voltage, int duration_ms, int sampleRate_ms) {
    printf("Collecting data with voltage: %.2f for %d ms\n", voltage, duration_ms);
    
    // Clear previous data
    velocityHistory.clear();
    timeHistory.clear();
    
    // Apply voltage to motor
    motor.move(voltage);
    
    // Collect data
    uint32_t startTime = pros::millis();
    uint32_t lastSampleTime = startTime;
    
    while (pros::millis() - startTime < duration_ms) {
        uint32_t currentTime = pros::millis();
        
        // Sample at specified rate
        if (currentTime - lastSampleTime >= sampleRate_ms) {
            double timestamp = (currentTime - startTime) / 1000.0; // Convert to seconds
            double velocity = motor.get_actual_velocity();
            
            velocityHistory.push_back(velocity);
            timeHistory.push_back(timestamp);
            
            // Keep only the last HISTORY_SIZE samples
            if (velocityHistory.size() > HISTORY_SIZE) {
                velocityHistory.erase(velocityHistory.begin());
                timeHistory.erase(timeHistory.begin());
            }
            
            lastSampleTime = currentTime;
            
            // Calculate acceleration if we have enough data
            double acceleration = 0.0;
            if (velocityHistory.size() >= 2) {
                acceleration = calculateAcceleration(velocityHistory, timeHistory);
            }
            
            // Add data point to system identification
            if (velocityHistory.size() >= 2) {
                sysId.addDataPoint(voltage, velocity, acceleration, timestamp);
            }
            
            printf("Time: %.2fs, Velocity: %.2f RPM, Acceleration: %.2f RPM/s\n", 
                   timestamp, velocity, acceleration);
        }
        
        pros::delay(5); // Small delay to prevent overwhelming the system
    }
    
    // Stop motor
    motor.move(0);
    printf("Data collection complete. Total points: %zu\n", sysId.getDataPointCount());
}

/**
 * @brief Run a complete motor characterization test
 * @param motor The motor to characterize
 */
void runMotorCharacterization(pros::Motor& motor) {
    printf("Starting motor characterization...\n");
    
    // Clear any existing data
    sysId.clearData();
    
    // Test voltages (from low to high)
    std::vector<double> testVoltages = {-100, -80, -60, -40, -20, 0, 20, 40, 60, 80, 100};
    
    for (double voltage : testVoltages) {
        // Skip zero voltage as it doesn't provide useful data
        if (std::abs(voltage) < 5) continue;
        
        // Collect data for each voltage level
        collectMotorData(motor, voltage, 2000, 50); // 2 seconds, 50ms sample rate
        
        // Wait between tests
        pros::delay(1000);
    }
    
    printf("Characterization complete. Attempting system identification...\n");
    
    // Perform system identification
    bool success = sysId.identify(true, true); // Include both static friction and acceleration
    
    if (success) {
        printf("System identification successful!\n");
        sysId.printResults();
        
        // Export data to CSV for external analysis
        if (sysId.exportToCSV("/usd/motor_data.csv")) {
            printf("Data exported to /usd/motor_data.csv\n");
        }
    } else {
        printf("System identification failed. Check data quality.\n");
    }
}

/**
 * @brief Test the identified feedforward constants
 * @param motor The motor to test
 * @param targetVelocity Target velocity to test
 */
void testFeedforwardConstants(pros::Motor& motor, double targetVelocity) {
    if (!sysId.isSystemIdentified()) {
        printf("System not identified yet. Run characterization first.\n");
        return;
    }
    
    FeedforwardConstants constants = sysId.getConstants();
    printf("Testing feedforward constants with target velocity: %.2f RPM\n", targetVelocity);
    
    // Calculate feedforward voltage
    double feedforwardVoltage = constants.calculate(targetVelocity, 0.0); // Assume zero acceleration for steady state
    
    printf("Feedforward voltage: %.2f\n", feedforwardVoltage);
    
    // Apply feedforward voltage
    motor.move(feedforwardVoltage);
    
    // Monitor performance
    uint32_t startTime = pros::millis();
    while (pros::millis() - startTime < 5000) { // Test for 5 seconds
        double actualVelocity = motor.get_actual_velocity();
        double error = targetVelocity - actualVelocity;
        
        printf("Target: %.2f RPM, Actual: %.2f RPM, Error: %.2f RPM\n", 
               targetVelocity, actualVelocity, error);
        
        pros::delay(100);
    }
    
    motor.move(0);
    printf("Feedforward test complete.\n");
}

/**
 * @brief Example usage in autonomous mode
 */
void autonomous() {
    // Create motor object (adjust port number as needed)
    pros::Motor testMotor(1);
    
    // Run motor characterization
    runMotorCharacterization(testMotor);
    
    // Test the identified constants
    testFeedforwardConstants(testMotor, 50.0); // Test at 50 RPM
}

/**
 * @brief Example usage in operator control mode
 */
void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    pros::Motor testMotor(1);
    
    bool characterizationStarted = false;
    
    while (true) {
        // Press A to start characterization
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A) && !characterizationStarted) {
            characterizationStarted = true;
            printf("Starting motor characterization...\n");
            runMotorCharacterization(testMotor);
        }
        
        // Press B to test feedforward at 50 RPM
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B) && sysId.isSystemIdentified()) {
            testFeedforwardConstants(testMotor, 50.0);
        }
        
        // Press X to print current results
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            if (sysId.isSystemIdentified()) {
                sysId.printResults();
            } else {
                printf("System not identified yet. Run characterization first.\n");
            }
        }
        
        // Press Y to clear data and start over
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            sysId.clearData();
            characterizationStarted = false;
            printf("Data cleared. Ready for new characterization.\n");
        }
        
        // Display status on LCD
        pros::lcd::print(0, "Motor Characterization Tool");
        pros::lcd::print(1, "A: Start Char, B: Test FF");
        pros::lcd::print(2, "X: Print Results, Y: Clear");
        pros::lcd::print(3, "Data Points: %zu", sysId.getDataPointCount());
        
        if (sysId.isSystemIdentified()) {
            FeedforwardConstants constants = sysId.getConstants();
            pros::lcd::print(4, "kS: %.2f, kV: %.2f, kA: %.2f", 
                           constants.kS, constants.kV, constants.kA);
            pros::lcd::print(5, "R^2: %.3f", sysId.getRSquared());
        } else {
            pros::lcd::print(4, "System not identified");
            pros::lcd::print(5, "Run characterization first");
        }
        
        pros::delay(20);
    }
}
