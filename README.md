# Motor Characterization System

This project provides a comprehensive system identification tool for characterizing VEX motors and identifying feedforward constants for improved motor control.

## Overview

The motor characterization system uses least squares regression to identify three key feedforward constants:

- **kS (Static Friction)**: Compensates for static friction in the motor/gearbox
- **kV (Velocity)**: Relates voltage to steady-state velocity
- **kA (Acceleration)**: Relates voltage to acceleration

The system implements the feedforward model:
```
V = kS * sign(v) + kV * v + kA * a
```

Where:
- V = Applied voltage
- v = Target velocity
- a = Target acceleration

## Features

- **Eigen Integration**: Uses Eigen library for robust matrix operations and least squares regression
- **Least Squares Regression**: Robust system identification using QR decomposition
- **Real-time Data Collection**: Collect motor data during operation
- **CSV Export**: Export data for external analysis
- **R-squared Analysis**: Measure the quality of the fit
- **Interactive Testing**: Test identified constants in real-time

## Files

- `include/system_identification.hpp`: Main header file with class definitions
- `src/system_identification.cpp`: Implementation of matrix operations and system identification
- `src/motor_characterization_example.cpp`: Example usage and data collection routines

## Usage

### Basic Usage

```cpp
#include "system_identification.hpp"
using namespace motor_characterization;

// Create system identification object
SystemIdentification sysId;

// Add data points (voltage, velocity, acceleration, timestamp)
sysId.addDataPoint(50.0, 100.0, 10.0, 1.0);
sysId.addDataPoint(75.0, 150.0, 15.0, 2.0);
// ... add more data points

// Perform system identification
bool success = sysId.identify(true, true); // Include static friction and acceleration

if (success) {
    // Get identified constants
    FeedforwardConstants constants = sysId.getConstants();
    printf("kS: %.4f, kV: %.4f, kA: %.4f\n", 
           constants.kS, constants.kV, constants.kA);
    
    // Use constants for feedforward control
    double targetVelocity = 100.0; // RPM
    double targetAcceleration = 0.0; // RPM/s
    double feedforwardVoltage = constants.calculate(targetVelocity, targetAcceleration);
}
```

### Complete Motor Characterization

The example file provides a complete motor characterization routine:

```cpp
// Create motor object
pros::Motor testMotor(1);

// Run complete characterization
runMotorCharacterization(testMotor);

// Test the identified constants
testFeedforwardConstants(testMotor, 50.0);
```

### Interactive Control

In operator control mode, use the controller buttons:

- **A Button**: Start motor characterization
- **B Button**: Test feedforward constants at 50 RPM
- **X Button**: Print current identification results
- **Y Button**: Clear data and start over

## Data Collection Process

1. **Setup**: Ensure the motor is properly connected and can move freely
2. **Characterization**: Run through a series of voltage levels (-100 to +100)
3. **Data Collection**: For each voltage level, collect velocity and acceleration data
4. **Identification**: Use least squares regression to identify constants
5. **Validation**: Test the identified constants with different target velocities

## Matrix Operations

The system uses Eigen library for all matrix operations:

- Matrix multiplication, addition, subtraction
- Transpose and inverse operations
- Determinant calculation
- QR decomposition for least squares
- SVD for singular value decomposition
- Built-in numerical stability

## Mathematical Background

### Least Squares Regression

The system identification uses QR decomposition for numerical stability:
```
β = (X^T * X)^(-1) * X^T * y
```

Or more robustly using QR decomposition:
```
X = QR
β = R^(-1) * Q^T * y
```

Where:
- X = Design matrix with features (sign(v), v, a)
- y = Response vector (applied voltages)
- β = Parameter vector (kS, kV, kA)
- Q, R = QR decomposition of X

### R-squared Calculation

The quality of the fit is measured using R-squared:
```
R² = 1 - (RSS / TSS)
```

Where:
- RSS = Residual Sum of Squares
- TSS = Total Sum of Squares

## Best Practices

1. **Data Quality**: Ensure motor is not stalled and data is clean
2. **Sample Rate**: Use appropriate sample rates (20-50ms recommended)
3. **Voltage Range**: Test across the full operating range
4. **Steady State**: Allow motor to reach steady state at each voltage level
5. **Validation**: Always test identified constants before using in control

## Troubleshooting

### Poor R-squared Values
- Check for motor stalling or mechanical issues
- Ensure data collection is not interrupted
- Verify voltage and velocity measurements are accurate

### Identification Fails
- Ensure at least 3 data points are collected
- Check that the design matrix is not singular
- Verify data spans a reasonable range of velocities

### Unrealistic Constants
- Check units (voltage should be -127 to 127, velocity in RPM)
- Verify acceleration calculation is correct
- Ensure data collection timing is accurate

## Example Output

```
=== System Identification Results ===
Data points: 45
R-squared: 0.9876

Feedforward Constants:
kS (Static Friction): 2.3456
kV (Velocity): 0.1234
kA (Acceleration): 0.0012

Model: V = kS*sign(v) + kV*v + kA*a
=====================================
```

## Integration with Control Systems

Once identified, the feedforward constants can be used in various control systems:

### PID + Feedforward
```cpp
double error = targetVelocity - actualVelocity;
double pidOutput = kP * error + kI * integral + kD * derivative;
double feedforwardOutput = constants.calculate(targetVelocity, targetAcceleration);
double totalOutput = pidOutput + feedforwardOutput;
```

### Pure Feedforward
```cpp
double output = constants.calculate(targetVelocity, targetAcceleration);
motor.move(output);
```

## License

This project is part of the motor characterization system for VEX robotics. Use in accordance with VEX and PROS licensing terms.
