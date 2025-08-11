#include "main.h"
#include "system_identification.hpp"
#include <vector>
#include <cmath>
#include <iostream>
#include <atomic>
#include <algorithm>

using namespace motor_characterization;

// Motor to characterize (adjust port as needed)
pros::Motor characterizationMotor(1);
static std::atomic<bool> startRequested{false};
static std::atomic<bool> consistencyTestRequested{false};

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
        2000,      // Start at zero
        6000,   // Jump to high positive
        2000,      // Back to zero (deceleration)
        -6000,  // Jump to high negative
        0,      // Back to zero (deceleration)
        12000,  // Jump to max positive
        0,      // Back to zero (deceleration)
        -12000, // Jump to max negative
        1000,      // Back to zero (deceleration)
        3000,   // Medium positive
        -1000,      // Back to zero
        -3000   // Medium negative
    };
    
    // Calculate time per voltage level (20 seconds total)
    int totalVoltages = testVoltages.size();
    int timePerVoltage = 20000 / totalVoltages; // 20 seconds / number of voltages
    
    // LCD: show we're starting (no clears)
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
    pros::lcd::print(0, "Analyzing Data...");
    pros::lcd::print(1, "Total Points: %zu", motorSysId.getDataPointCount());

    // Debug: Print some data statistics before identification
    const auto& dataPoints = motorSysId.getDataPoints();
    if (!dataPoints.empty()) {
        double minVoltage = dataPoints[0].voltage, maxVoltage = dataPoints[0].voltage;
        double minVelocity = dataPoints[0].velocity, maxVelocity = dataPoints[0].velocity;
        double minAccel = dataPoints[0].acceleration, maxAccel = dataPoints[0].acceleration;
        
        for (const auto& point : dataPoints) {
            minVoltage = std::min(minVoltage, point.voltage);
            maxVoltage = std::max(maxVoltage, point.voltage);
            minVelocity = std::min(minVelocity, point.velocity);
            maxVelocity = std::max(maxVelocity, point.velocity);
            minAccel = std::min(minAccel, point.acceleration);
            maxAccel = std::max(maxAccel, point.acceleration);
        }
        
        printf("\nData Statistics:\n");
        printf("Voltage range: %.2f to %.2f V\n", minVoltage, maxVoltage);
        printf("Velocity range: %.1f to %.1f RPM\n", minVelocity, maxVelocity);
        printf("Acceleration range: %.1f to %.1f RPM/s\n", minAccel, maxAccel);
        printf("Data points: %zu\n", dataPoints.size());
    }
    
    bool success = motorSysId.identify(true, true); // Include static friction and acceleration

    if (success) {
        FeedforwardConstants constants = motorSysId.getConstants();
        
        // Print results to terminal
        printf("\n=== MOTOR CHARACTERIZATION RESULTS ===\n");
        printf("Data points collected: %zu\n", motorSysId.getDataPointCount());
        printf("R-squared (fit quality): %.4f\n", motorSysId.getRSquared());
        printf("\nFeedforward Constants:\n");
        printf("kS (Static Friction): %.4f V\n", constants.kS);
        printf("kV (Velocity): %.4f V/RPM\n", constants.kV);
        printf("kA (Acceleration): %.6f V/(RPM/s)\n", constants.kA);
        printf("\nModel: V = kS*sign(v) + kV*v + kA*a\n");
        
        // Debug: Check for negative kS and explain possible causes
        if (constants.kS < 0) {
            printf("\n⚠️  WARNING: Negative kS detected!\n");
            printf("Possible causes:\n");
            printf("1. Motor has very low friction (good motor)\n");
            printf("2. Data collection issues (noise, timing)\n");
            printf("3. Motor not properly loaded/connected\n");
            printf("4. Insufficient data at low velocities\n");
            printf("5. Numerical issues in regression\n");
            printf("\nRecommendations:\n");
            printf("- Check motor connection and loading\n");
            printf("- Ensure motor can spin freely\n");
            printf("- Try longer test duration\n");
            printf("- Check for electrical noise\n");
        }
        
        // Calculate and print some useful metrics
        double maxVoltage = 12.0;
        double maxVelocity = (maxVoltage - constants.kS) / constants.kV;
        double voltage100 = constants.calculate(100.0, 0.0);
        
        printf("\nCalculated Metrics:\n");
        printf("Max velocity (at 12V): %.1f RPM\n", maxVelocity);
        printf("Voltage for 100 RPM: %.2f V\n", voltage100);
        printf("=====================================\n\n");
        
        // Also show on LCD
        pros::lcd::print(0, "kS: %.2f kV: %.3f", constants.kS, constants.kV);
        pros::lcd::print(1, "kA: %.4f R^2: %.3f", constants.kA, motorSysId.getRSquared());
        pros::lcd::print(2, "Max Vel: %.0f RPM", maxVelocity);
        pros::lcd::print(3, "100RPM: %.1fV", voltage100);
        pros::lcd::print(4, "Points: %zu", motorSysId.getDataPointCount());
        pros::lcd::print(5, "Press center to retest");
    } else {
        printf("\n=== CHARACTERIZATION FAILED ===\n");
        printf("Not enough valid data points for identification.\n");
        printf("Make sure motor is connected and can spin freely.\n");
        printf("=====================================\n\n");
        
        pros::lcd::print(0, "Identification failed");
        pros::lcd::print(1, "Check terminal for details");
        pros::lcd::print(2, "Press center to retry");
    }
}

/**
 * @brief Run 10 consecutive tests and analyze consistency
 */
void runConsistencyTest() {
    std::vector<FeedforwardConstants> results;
    std::vector<double> rSquaredValues;
    
    printf("\n=== STARTING CONSISTENCY TEST (5 runs) ===\n");
    pros::lcd::print(0, "Consistency Test");
    pros::lcd::print(1, "5 consecutive tests");
    
    for (int test = 1; test <= 5; ++test) {
        printf("\n--- Test %d/5 ---\n", test);
        pros::lcd::print(0, "Test %d/5", test);
        
        // Create fresh system identification object for each test
        SystemIdentification motorSysId;
        
        // Define voltage test points (same as single test)
        std::vector<int> testVoltages = {
            2000,      // Start at zero
        6000,   // Jump to high positive
        2000,      // Back to zero (deceleration)
        -6000,  // Jump to high negative
        0,      // Back to zero (deceleration)
        12000,  // Jump to max positive
        0,      // Back to zero (deceleration)
        -12000, // Jump to max negative
        1000,      // Back to zero (deceleration)
        3000,   // Medium positive
        -1000,      // Back to zero
        -3000   // Medium negative
        };
        
        int totalVoltages = testVoltages.size();
        int timePerVoltage = 20000 / totalVoltages;
        
        // Collect data for each voltage level
        for (size_t i = 0; i < testVoltages.size(); ++i) {
            int voltage = testVoltages[i];
            pros::lcd::print(1, "Voltage %d/%d", i + 1, totalVoltages);
            
            double previousVelocity = 0.0;
            double previousTime = 0.0;
            
            characterizationMotor.move_voltage(voltage);
            
            uint32_t startTime = pros::millis();
            while (pros::millis() - startTime < timePerVoltage) {
                double currentTime = (pros::millis() - startTime) / 1000.0;
                double currentVelocity = characterizationMotor.get_actual_velocity();
                
                if (previousTime > 0) {
                    double dt = currentTime - previousTime;
                    if (dt > 0.001) {
                        double acceleration = (currentVelocity - previousVelocity) / dt;
                        double voltageV = voltage / 1000.0;
                        motorSysId.addDataPoint(voltageV, currentVelocity, acceleration, currentTime);
                    }
                }
                
                previousVelocity = currentVelocity;
                previousTime = currentTime;
                pros::delay(10);
            }
            
            characterizationMotor.move_voltage(0);
        }
        
        // Perform identification
        bool success = motorSysId.identify(true, true);
        
        if (success) {
            FeedforwardConstants constants = motorSysId.getConstants();
            results.push_back(constants);
            rSquaredValues.push_back(motorSysId.getRSquared());
            
            printf("Test %d: kS=%.3f, kV=%.4f, kA=%.5f, R²=%.3f\n", 
                   test, constants.kS, constants.kV, constants.kA, motorSysId.getRSquared());
        } else {
            printf("Test %d: FAILED\n", test);
        }
        
        // Brief pause between tests
        pros::delay(500);
    }
    
    // Analyze consistency
    if (results.size() >= 3) {
        printf("\n=== CONSISTENCY ANALYSIS ===\n");
        printf("Successful tests: %zu/5\n", results.size());
        
        // Calculate statistics for each parameter
        std::vector<double> kS_values, kV_values, kA_values;
        for (const auto& result : results) {
            kS_values.push_back(result.kS);
            kV_values.push_back(result.kV);
            kA_values.push_back(result.kA);
        }
        
        // Calculate means and standard deviations
        auto calculateStats = [](const std::vector<double>& values) {
            double sum = 0.0, sumSq = 0.0;
            for (double val : values) {
                sum += val;
                sumSq += val * val;
            }
            double mean = sum / values.size();
            double variance = (sumSq / values.size()) - (mean * mean);
            double stdDev = sqrt(variance);
            return std::make_pair(mean, stdDev);
        };
        
        auto [kS_mean, kS_std] = calculateStats(kS_values);
        auto [kV_mean, kV_std] = calculateStats(kV_values);
        auto [kA_mean, kA_std] = calculateStats(kA_values);
        
        // Calculate coefficients of variation (CV = std/mean)
        double kS_cv = kS_std / fabs(kS_mean);
        double kV_cv = kV_std / fabs(kV_mean);
        double kA_cv = kA_std / fabs(kA_mean);
        
        printf("\nParameter Statistics:\n");
        printf("kS: %.4f ± %.4f V (CV: %.1f%%)\n", kS_mean, kS_std, kS_cv * 100);
        printf("kV: %.4f ± %.4f V/RPM (CV: %.1f%%)\n", kV_mean, kV_std, kV_cv * 100);
        printf("kA: %.6f ± %.6f V/(RPM/s) (CV: %.1f%%)\n", kA_mean, kA_std, kA_cv * 100);
        
        // Consistency assessment
        printf("\nConsistency Assessment:\n");
        if (kS_cv < 0.05 && kV_cv < 0.05 && kA_cv < 0.10) {
            printf("✅ EXCELLENT consistency (< 5%% variation)\n");
        } else if (kS_cv < 0.10 && kV_cv < 0.10 && kA_cv < 0.20) {
            printf("✅ GOOD consistency (< 10%% variation)\n");
        } else if (kS_cv < 0.20 && kV_cv < 0.20 && kA_cv < 0.30) {
            printf("⚠️  FAIR consistency (< 20%% variation)\n");
        } else {
            printf("❌ POOR consistency (> 20%% variation)\n");
        }
        
        // Display on LCD
        pros::lcd::print(0, "Consistency: %.1f%%", (kS_cv + kV_cv + kA_cv) * 100 / 3);
        pros::lcd::print(1, "kS: %.3f±%.3f", kS_mean, kS_std);
        pros::lcd::print(2, "kV: %.4f±%.4f", kV_mean, kV_std);
        pros::lcd::print(3, "kA: %.5f±%.5f", kA_mean, kA_std);
        pros::lcd::print(4, "Tests: %zu/5", results.size());
        pros::lcd::print(5, "Press center to retest");
        
    } else {
        printf("\n❌ INSUFFICIENT DATA FOR CONSISTENCY ANALYSIS\n");
        printf("Need at least 3 successful tests, got %zu\n", results.size());
        
        pros::lcd::print(0, "Insufficient data");
        pros::lcd::print(1, "Only %zu/5 tests passed", results.size());
        pros::lcd::print(2, "Check motor connection");
        pros::lcd::print(3, "Press center to retry");
    }
    
    printf("\n=====================================\n\n");
}

/**
 * @brief Display motor characteristics on LCD
 */
void displayMotorCharacteristics() {
    // Create local system identification object
    SystemIdentification motorSysId;
    
    if (!motorSysId.isSystemIdentified()) {
        pros::lcd::print(0, "No Characterization Data");
        pros::lcd::print(1, "Press center to start");
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
    pros::lcd::print(4, "Max Vel: %.0f RPM", maxVelocity);
    
    // Show voltage for 100 RPM
    double voltage100 = constants.calculate(100.0, 0.0);
    pros::lcd::print(5, "100RPM: %.1fV", voltage100);
}

/**
 * A callback function for LLEMU's center button.
 */
void on_center_button() {
    startRequested = true;
}

void on_right_button() {
    consistencyTestRequested = true;
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 */
void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(0, "Motor Characterization");
    pros::lcd::set_text(1, "Center: Single test");
    pros::lcd::set_text(2, "Right: 5 tests");
    
    pros::lcd::register_btn1_cb(on_center_button);
    pros::lcd::register_btn2_cb(on_right_button);

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
    bool isCharacterizing = false;

    while (true) {
        if (startRequested && !isCharacterizing) {
            isCharacterizing = true;
            startRequested = false;
            runMotorCharacterization();
            isCharacterizing = false;
        }
        
        if (consistencyTestRequested && !isCharacterizing) {
            isCharacterizing = true;
            consistencyTestRequested = false;
            runConsistencyTest();
            isCharacterizing = false;
        }

        pros::delay(20);
    }
}