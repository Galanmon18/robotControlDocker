#include "dependecies/abdomen_compliance_controller.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

AbdomenComplianceController::AbdomenComplianceController(double K_abd, double Kp, double Ki)
    : K_abd_(K_abd), Kp_(Kp), Ki_(Ki),
      theta_x_int_(0.0), theta_y_int_(0.0),
      theta_x_des_(0.0), theta_y_des_(0.0),
      theta_x_cur_(0.0), theta_y_cur_(0.0)
{}

void AbdomenComplianceController::reset(){
    theta_x_int_ = 0.0;
    theta_y_int_ = 0.0;
}

void AbdomenComplianceController::computeDesiredAngles(double Fx, double Fy,
                                                        double L_total, double rho){
    // Lever arm: internal instrument length (clamp to avoid division by ~zero)
    const double L_internal = std::max(L_total - rho, L_MIN_INTERNAL);

    // Compliance law: θ = F / (K * L_internal)
    // A force Fx on the abdominal wall requires a tilt θx to relieve it.
    // Larger internal length → more mechanical advantage → less tilt needed.
    theta_x_des_ = -Fy / (K_abd_ * L_internal);
    theta_y_des_ = Fx / (K_abd_ * L_internal);

    // Clamp to safe range
    theta_x_des_ = std::max(-THETA_MAX, std::min(THETA_MAX, theta_x_des_));
    theta_y_des_ = std::max(-THETA_MAX, std::min(THETA_MAX, theta_y_des_));
}

void AbdomenComplianceController::extractCurrentAngles(const Eigen::Matrix3d& R){
    // R.col(2) = Z-axis of TCP in world frame.
    // For a nominally downward instrument, deviations give tilt angles.
    // Small-angle approximation: sin(θ) ≈ θ for |θ| < ~20°
    theta_x_cur_ =  R(2, 1);  //  R[2,1] ≈ sin(θx) ≈ θx
    theta_y_cur_ = -R(2, 0);  // -R[2,0] ≈ sin(θy) ≈ θy
}

std::array<double,2> AbdomenComplianceController::compute(double Fx, double Fy,
                                                           double rho, double L_total){
    // Step 1: compliance law
    computeDesiredAngles(Fx, Fy, L_total, rho);

    // Step 2: angular error
    const double e_x = theta_x_des_;
    const double e_y = theta_y_des_;

    // Step 4: PI with anti-windup
    theta_x_int_ += e_x * DT;
    theta_y_int_ += e_y * DT;
    theta_x_int_  = std::max(-0.3, std::min(0.3, theta_x_int_));
    theta_y_int_  = std::max(-0.3, std::min(0.3, theta_y_int_));

    const double wx = Kp_ * e_x + Ki_ * theta_x_int_;
    const double wy = Kp_ * e_y + Ki_ * theta_y_int_;

    std::cout << "Compliance | θx_des=" << theta_x_des_ << " θx_cur=" << theta_x_cur_
              << " | wx=" << wx << " wy=" << wy << std::endl;

    return {wx, wy};
}

// ODR definitions (C++14)
constexpr double AbdomenComplianceController::THETA_MAX;
//constexpr double AbdomenComplianceController::INT_MAX;
constexpr double AbdomenComplianceController::L_MIN_INTERNAL;
constexpr double AbdomenComplianceController::DT;
