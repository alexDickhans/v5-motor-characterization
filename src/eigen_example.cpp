#include "system_identification.hpp"
#include "main.h"
#include <iostream>

using namespace motor_characterization;

/**
 * @brief Example demonstrating Eigen-based system identification
 */
void eigenExample() {
    SystemIdentification sysId;
    
    // Add some sample data points
    // Format: (voltage, velocity, acceleration, timestamp)
    sysId.addDataPoint(20.0, 50.0, 5.0, 1.0);
    sysId.addDataPoint(40.0, 100.0, 10.0, 2.0);
    sysId.addDataPoint(60.0, 150.0, 15.0, 3.0);
    sysId.addDataPoint(80.0, 200.0, 20.0, 4.0);
    sysId.addDataPoint(100.0, 250.0, 25.0, 5.0);
    
    // Add some negative velocity data for static friction identification
    sysId.addDataPoint(-20.0, -50.0, -5.0, 6.0);
    sysId.addDataPoint(-40.0, -100.0, -10.0, 7.0);
    sysId.addDataPoint(-60.0, -150.0, -15.0, 8.0);
    
    printf("Added %zu data points\n", sysId.getDataPointCount());
    
    // Perform system identification
    bool success = sysId.identify(true, true); // Include static friction and acceleration
    
    if (success) {
        printf("System identification successful!\n");
        sysId.printResults();
        
        // Get the identified constants
        FeedforwardConstants constants = sysId.getConstants();
        
        // Test the model with some predictions
        printf("\nTesting predictions:\n");
        double testVelocities[] = {75.0, 125.0, 175.0};
        double testAccelerations[] = {7.5, 12.5, 17.5};
        
        for (int i = 0; i < 3; ++i) {
            double predictedVoltage = constants.calculate(testVelocities[i], testAccelerations[i]);
            printf("Velocity: %.1f RPM, Acceleration: %.1f RPM/s -> Predicted Voltage: %.2f\n",
                   testVelocities[i], testAccelerations[i], predictedVoltage);
        }
        
        // Get design matrix and response vector for external analysis
        Eigen::MatrixXd X = sysId.getDesignMatrix(true, true);
        Eigen::VectorXd y = sysId.getResponseVector();
        
        printf("\nDesign matrix shape: %ld x %ld\n", X.rows(), X.cols());
        printf("Response vector size: %ld\n", y.size());
        
        // Show first few rows of design matrix
        printf("\nFirst 3 rows of design matrix:\n");
        for (int i = 0; i < std::min(3, (int)X.rows()); ++i) {
            printf("Row %d: [%.2f, %.2f, %.2f]\n", i, X(i,0), X(i,1), X(i,2));
        }
        
    } else {
        printf("System identification failed!\n");
    }
}

/**
 * @brief Example showing how to use the system identification in autonomous mode
 */
void autonomous() {
    printf("Running Eigen-based system identification example...\n");
    eigenExample();
}

/**
 * @brief Example showing how to use the system identification in operator control mode
 */
void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    
    bool exampleRun = false;
    
    while (true) {
        // Press A to run the example
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A) && !exampleRun) {
            exampleRun = true;
            printf("Running Eigen example...\n");
            eigenExample();
        }
        
        // Press B to reset
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            exampleRun = false;
            printf("Reset - ready to run example again\n");
        }
        
        // Display status on LCD
        pros::lcd::print(0, "Eigen System ID Example");
        pros::lcd::print(1, "A: Run Example");
        pros::lcd::print(2, "B: Reset");
        pros::lcd::print(3, "Status: %s", exampleRun ? "Complete" : "Ready");
        
        pros::delay(20);
    }
}
