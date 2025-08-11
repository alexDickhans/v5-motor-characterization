#ifndef SYSTEM_IDENTIFICATION_HPP
#define SYSTEM_IDENTIFICATION_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include "Eigen/Dense"
#include "api.h"

namespace motor_characterization {

/**
 * @brief Data point structure for system identification
 */
struct DataPoint {
    double voltage;      // Input voltage (-127 to 127)
    double velocity;     // Measured velocity (RPM)
    double acceleration; // Measured acceleration (RPM/s)
    double timestamp;    // Timestamp of measurement
    
    DataPoint(double v, double vel, double acc, double t) 
        : voltage(v), velocity(vel), acceleration(acc), timestamp(t) {}
};

/**
 * @brief Feedforward constants structure
 */
struct FeedforwardConstants {
    double kS;  // Static friction constant
    double kV;  // Velocity feedforward constant
    double kA;  // Acceleration feedforward constant
    
    FeedforwardConstants(double s = 0.0, double v = 0.0, double a = 0.0) 
        : kS(s), kV(v), kA(a) {}
    
    // Calculate feedforward output
    double calculate(double velocity, double acceleration) const {
        return kS * (velocity > 0 ? 1.0 : -1.0) + kV * velocity + kA * acceleration;
    }
};

/**
 * @brief System identification class for motor feedforward constants
 * 
 * This class implements least squares regression to identify feedforward constants
 * for motor control systems using Eigen library. It can handle:
 * - Static friction (kS)
 * - Velocity feedforward (kV) 
 * - Acceleration feedforward (kA)
 */
class SystemIdentification {
private:
    std::vector<DataPoint> dataPoints;
    FeedforwardConstants constants;
    double rSquared;
    bool isIdentified;

public:
    SystemIdentification() : rSquared(0.0), isIdentified(false) {}

    /**
     * @brief Add a data point to the identification dataset
     * @param voltage Input voltage to the motor
     * @param velocity Measured velocity
     * @param acceleration Measured acceleration
     * @param timestamp Timestamp of measurement
     */
    void addDataPoint(double voltage, double velocity, double acceleration, double timestamp) {
        dataPoints.emplace_back(voltage, velocity, acceleration, timestamp);
        isIdentified = false; // Reset identification when new data is added
    }

    /**
     * @brief Add a data point using the DataPoint structure
     * @param point Data point to add
     */
    void addDataPoint(const DataPoint& point) {
        dataPoints.push_back(point);
        isIdentified = false;
    }

    /**
     * @brief Clear all data points
     */
    void clearData() {
        dataPoints.clear();
        isIdentified = false;
    }

    /**
     * @brief Get the number of data points
     * @return Number of data points
     */
    size_t getDataPointCount() const {
        return dataPoints.size();
    }

    /**
     * @brief Perform system identification using least squares regression
     * @param includeStaticFriction Whether to include static friction term
     * @param includeAcceleration Whether to include acceleration feedforward term
     * @return True if identification was successful
     */
    bool identify(bool includeStaticFriction = true, bool includeAcceleration = true);

    /**
     * @brief Get the identified feedforward constants
     * @return Feedforward constants
     */
    FeedforwardConstants getConstants() const {
        return constants;
    }

    /**
     * @brief Get the R-squared value of the fit
     * @return R-squared value (0 to 1, higher is better)
     */
    double getRSquared() const {
        return rSquared;
    }

    /**
     * @brief Check if system has been identified
     * @return True if identification has been performed
     */
    bool isSystemIdentified() const {
        return isIdentified;
    }

    /**
     * @brief Calculate the predicted voltage for given velocity and acceleration
     * @param velocity Target velocity
     * @param acceleration Target acceleration
     * @return Predicted voltage
     */
    double predictVoltage(double velocity, double acceleration) const {
        if (!isIdentified) return 0.0;
        return constants.calculate(velocity, acceleration);
    }

    /**
     * @brief Calculate the error between predicted and actual voltage
     * @param actualVoltage Actual voltage applied
     * @param velocity Measured velocity
     * @param acceleration Measured acceleration
     * @return Error in voltage
     */
    double calculateError(double actualVoltage, double velocity, double acceleration) const {
        return actualVoltage - predictVoltage(velocity, acceleration);
    }

    /**
     * @brief Print identification results to console
     */
    void printResults() const;

    /**
     * @brief Export data to CSV format for external analysis
     * @param filename Output filename
     * @return True if export was successful
     */
    bool exportToCSV(const std::string& filename) const;

    /**
     * @brief Get data points for external analysis
     * @return Vector of data points
     */
    const std::vector<DataPoint>& getDataPoints() const {
        return dataPoints;
    }

    /**
     * @brief Get the design matrix for external analysis
     * @param includeStaticFriction Whether to include static friction term
     * @param includeAcceleration Whether to include acceleration feedforward term
     * @return Design matrix as Eigen::MatrixXd
     */
    Eigen::MatrixXd getDesignMatrix(bool includeStaticFriction = true, bool includeAcceleration = true) const;

    /**
     * @brief Get the response vector for external analysis
     * @return Response vector as Eigen::VectorXd
     */
    Eigen::VectorXd getResponseVector() const;

private:
    /**
     * @brief Build the design matrix for least squares regression
     * @param includeStaticFriction Whether to include static friction term
     * @param includeAcceleration Whether to include acceleration feedforward term
     * @return Design matrix
     */
    Eigen::MatrixXd buildDesignMatrix(bool includeStaticFriction, bool includeAcceleration) const;

    /**
     * @brief Build the response vector for least squares regression
     * @return Response vector
     */
    Eigen::VectorXd buildResponseVector() const;

    /**
     * @brief Calculate R-squared value
     * @param predicted Predicted values
     * @param actual Actual values
     * @return R-squared value
     */
    double calculateRSquared(const Eigen::VectorXd& predicted, const Eigen::VectorXd& actual) const;
};

} // namespace motor_characterization

#endif // SYSTEM_IDENTIFICATION_HPP
