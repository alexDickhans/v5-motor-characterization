#include "system_identification.hpp"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <numeric>

namespace motor_characterization {

// SystemIdentification implementation
Eigen::MatrixXd SystemIdentification::buildDesignMatrix(bool includeStaticFriction, bool includeAcceleration) const {
    size_t numPoints = dataPoints.size();
    size_t numFeatures = 1; // Always include velocity
    if (includeStaticFriction) numFeatures++;
    if (includeAcceleration) numFeatures++;
    
    Eigen::MatrixXd X(numPoints, numFeatures);
    
    for (size_t i = 0; i < numPoints; ++i) {
        size_t col = 0;
        
        // Static friction term (sign of velocity)
        if (includeStaticFriction) {
            X(i, col++) = dataPoints[i].velocity > 0 ? 1.0 : -1.0;
        }
        
        // Velocity term
        X(i, col++) = dataPoints[i].velocity;
        
        // Acceleration term
        if (includeAcceleration) {
            X(i, col++) = dataPoints[i].acceleration;
        }
    }
    
    return X;
}

Eigen::VectorXd SystemIdentification::buildResponseVector() const {
    size_t numPoints = dataPoints.size();
    Eigen::VectorXd y(numPoints);
    
    for (size_t i = 0; i < numPoints; ++i) {
        y(i) = dataPoints[i].voltage;
    }
    
    return y;
}

double SystemIdentification::calculateRSquared(const Eigen::VectorXd& predicted, const Eigen::VectorXd& actual) const {
    if (predicted.size() != actual.size() || predicted.size() == 0) return 0.0;
    
    // Calculate mean of actual values
    double mean = actual.mean();
    
    // Calculate total sum of squares and residual sum of squares
    double tss = (actual.array() - mean).square().sum();
    double rss = (actual - predicted).squaredNorm();
    
    if (tss < 1e-10) return 0.0;
    return 1.0 - (rss / tss);
}

bool SystemIdentification::identify(bool includeStaticFriction, bool includeAcceleration) {
    if (dataPoints.size() < 3) {
        // Need at least 3 data points for meaningful identification
        return false;
    }
    
    try {
        // Build design matrix and response vector
        Eigen::MatrixXd X = buildDesignMatrix(includeStaticFriction, includeAcceleration);
        Eigen::VectorXd y = buildResponseVector();
        
        // Solve least squares using Eigen's built-in solver
        // This uses QR decomposition which is more numerically stable than normal equations
        Eigen::VectorXd beta = X.colPivHouseholderQr().solve(y);
        
        // Check if the solution is valid
        if (!beta.allFinite()) {
            return false;
        }
        
        // Extract constants
        size_t idx = 0;
        if (includeStaticFriction) {
            constants.kS = beta(idx++);
        } else {
            constants.kS = 0.0;
        }
        
        constants.kV = beta(idx++);
        
        if (includeAcceleration) {
            constants.kA = beta(idx++);
        } else {
            constants.kA = 0.0;
        }
        
        // Calculate R-squared
        Eigen::VectorXd predicted = X * beta;
        rSquared = calculateRSquared(predicted, y);
        isIdentified = true;
        
        return true;
        
    } catch (...) {
        return false;
    }
}

Eigen::MatrixXd SystemIdentification::getDesignMatrix(bool includeStaticFriction, bool includeAcceleration) const {
    return buildDesignMatrix(includeStaticFriction, includeAcceleration);
}

Eigen::VectorXd SystemIdentification::getResponseVector() const {
    return buildResponseVector();
}

void SystemIdentification::printResults() const {
    if (!isIdentified) {
        printf("System has not been identified yet.\n");
        return;
    }
    
    printf("=== System Identification Results ===\n");
    printf("Data points: %zu\n", dataPoints.size());
    printf("R-squared: %.4f\n", rSquared);
    printf("\nFeedforward Constants:\n");
    printf("kS (Static Friction): %.4f\n", constants.kS);
    printf("kV (Velocity): %.4f\n", constants.kV);
    printf("kA (Acceleration): %.4f\n", constants.kA);
    printf("\nModel: V = kS*sign(v) + kV*v + kA*a\n");
    printf("=====================================\n");
}

bool SystemIdentification::exportToCSV(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    // Write header
    file << "Timestamp,Voltage,Velocity,Acceleration\n";
    
    // Write data
    for (const auto& point : dataPoints) {
        file << std::fixed << std::setprecision(6)
             << point.timestamp << ","
             << point.voltage << ","
             << point.velocity << ","
             << point.acceleration << "\n";
    }
    
    file.close();
    return true;
}

} // namespace motor_characterization
