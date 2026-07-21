#include "dependecies/tip_force_controller.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

TipForceController::TipForceController(double alpha_rise, double alpha_fall)
    : Fz_filt_(0.0),
      alpha_rise_(alpha_rise),
      alpha_fall_(alpha_fall),
      z_floor_(-std::numeric_limits<double>::infinity())
{}

void TipForceController::reset(){
    Fz_filt_ = 0.0;
    z_floor_ = -std::numeric_limits<double>::infinity();
}

void TipForceController::applyLimit(double Fz_raw, double& des_z, double K){

    // --- Step 1: Deadband ---
    const double Fz_db = (std::abs(Fz_raw) > DEADBAND) ? Fz_raw : 0.0;

    // --- Step 2: Asymmetric EWA filter ---
    // Use fast alpha when force is rising (new contact), slow alpha when falling.
    // This prevents the lag that causes the setpoint to overshoot:
    // fast rise → controller reacts before many small steps accumulate.
    const double alpha = (Fz_db > Fz_filt_) ? alpha_rise_ : alpha_fall_;
    Fz_filt_ = alpha * Fz_db + (1.0 - alpha) * Fz_filt_;

    // --- Step 3: Emergency ---
    if (Fz_filt_ > F_EMERGENCY) {
        z_floor_ = des_z + DZ_EMERGENCY;
        des_z    = z_floor_;
        std::cout << "⚠️ EMERGENCY: " << Fz_filt_ << " N | des_z=" << des_z << std::endl;
        return;
    }

    // --- Step 4: Integral limit with per-cycle cap ---
    if (Fz_filt_ > F_DES) {
        const double excess = Fz_filt_ - F_DES;

        // Cap per cycle: prevents the setpoint from racing ahead of the robot
        const double step = std::min(DZ_STEP_MAX, K * excess * DT);

        // Raise the floor and apply it
        z_floor_ = std::max(z_floor_, des_z + step);
        des_z    = std::max(des_z, z_floor_);

        std::cout << "Force limiter | Fz=" << Fz_filt_
                  << " N | excess=" << excess
                  << " N | step=" << step * 1000.0
                  << " mm | des_z=" << des_z << " m" << std::endl;
    }
    // Below F_DES: des_z holds at z_floor_ (or original goal if floor inactive)
    else if (z_floor_ > -std::numeric_limits<double>::infinity()) {
        des_z = std::max(des_z, z_floor_);  // Keep respecting the floor
    }
}

bool TipForceController::notifyNewGoal(double z_new, double& des_z){
    // Release the floor only if force is safe — operator can descend again
    if ((Fz_filt_ < F_DES) || z_new > des_z) {
        z_floor_ = -std::numeric_limits<double>::infinity();
        des_z    = z_new;
        return true;   // Goal accepted
    }
    // Force still high: hold the floor, ignore the lower goal
    std::cout << "New goal z=" << z_new
              << " rejected: Fz=" << Fz_filt_ << " N > F_DES. Hold floor." << std::endl;
    return false;      // Goal rejected — robot stays protected
}

// ODR definitions (C++14)
constexpr double TipForceController::DEADBAND;
constexpr double TipForceController::F_DES;
constexpr double TipForceController::F_EMERGENCY;
constexpr double TipForceController::DZ_EMERGENCY;
constexpr double TipForceController::DZ_STEP_MAX;
constexpr double TipForceController::DT;