#include "dependecies/abdomen_force_controller.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

AbdomenForceController::AbdomenForceController(double Kp, double Ki,
                                               double dt, double threshold,
                                               double v_max)
    : Kp_(Kp), Ki_(Ki), dt_(dt), threshold_(threshold), v_max_(v_max),
      active_(false), integral_{0.0, 0.0, 0.0}
{}

void AbdomenForceController::reset(){
    integral_ = {0.0, 0.0, 0.0};
    active_   = false;
}

std::vector<double> AbdomenForceController::compute(const std::vector<double>& f_abdomen){
    // F_desired = 0 (no contact wanted) → e_F = 0 - F = -F
    // Flee direction: velocity opposes the measured force
    std::vector<double> v_correction = {0.0, 0.0, 0.0};
    active_ = false;

    for (int i = 0; i < 3; ++i) {
        const double f = f_abdomen[i];

        // Deadband: ignore forces below threshold (sensor noise at rest)
        if (std::abs(f) < threshold_) {
            integral_[i] *= 0.9;  // Decay integrator when below threshold
            continue;
        }
        std::cout << "----------------------------------------ENTRO AbdomenForceController::compute"<< std::endl;
        active_ = true;

        // e_F = -f (desired = 0, flee direction opposes force)
        const double e_F = f;

        // Integral action (clamped to avoid windup)
        integral_[i] += e_F * dt_;
        integral_[i]  = std::max(-0.5, std::min(0.5, integral_[i]));

        // P(I) law: v = Kp*e + Ki*∫e
        double v = Kp_ * e_F + Ki_ * integral_[i];

        // Per-axis saturation
        v = std::max(-v_max_, std::min(v_max_, v));

        v_correction[i] = v; // I still don’t know why, if I read the force from the sensor, it’s v, but if it’s via the topic, it’s -v. I don’t know. BE CAREFUL
    }

    if (active_) {
        std::cout << "Abdomen force ctrl | F=[" << f_abdomen[0] << ", "
                  << f_abdomen[1] << ", " << f_abdomen[2] << "] N"
                  << " | v_corr=[" << v_correction[0] << ", "
                  << v_correction[1] << ", " << v_correction[2] << "] m/s"
                  << std::endl;
    }

    return v_correction;
}
