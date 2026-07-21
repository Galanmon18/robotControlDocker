#pragma once

/**
 * @file tip_force_controller.hpp
 * @brief Integral force limiter for instrument tip-tissue interaction.
 *
 * Two distinct thresholds:
 *   DEADBAND (0.3 N) : noise floor — forces below this are ignored.
 *   F_DES    (1.0 N) : desired force limit — above this, des_z is raised.
 *
 * Control law:
 *   des_z += min(DZ_STEP_MAX, K * excess * DT)   when Fz_filt > F_DES
 *
 * Key design decisions:
 *
 *   1. Asymmetric EWA filter:
 *      - alpha_rise (0.3) when force is increasing → fast response on contact
 *      - alpha_fall (0.05) when force is decreasing → slow decay, no oscillation
 *      This prevents the lag that causes the "pepinazo": when the robot descends
 *      into tissue, Fz_filt tracks the real force quickly, so the controller
 *      reacts before many small steps accumulate.
 *
 *   2. Per-cycle step cap (DZ_STEP_MAX):
 *      Limits how aggressively des_z rises each cycle regardless of excess.
 *      Prevents the setpoint from jumping far ahead of where the robot is.
 *
 *   3. Floor release on new goal:
 *      notifyNewGoal(z_new) must be called from cb_stitchCallback.
 *      If the new goal is lower AND Fz_filt < F_DES, the floor is released
 *      and des_z follows the operator's intent. If force is still above F_DES,
 *      the floor holds — the operator must wait for safe force before descending.
 */
class TipForceController {
public:
    explicit TipForceController(double alpha_rise = 0.3, double alpha_fall = 0.05);

    /**
     * @brief Filter force and apply integral limit to the Z setpoint.
     *        Call once per cycle.
     *
     * @param Fz_raw  Raw axial force (N). Positive = compression into tissue.
     * @param des_z   Desired Z (m). Modified in-place — raised if Fz > F_DES.
     * @param K       Integral gain m/(N·s). Default 0.005 (faster than before).
     */
    void applyLimit(double Fz_raw, double& des_z, double K = 0.005);

    /**
     * @brief Notify the controller that the operator has sent a new goal.
     *        If Fz_filt < F_DES, the floor is released so descent is allowed.
     *        If force is still high, the floor holds and des_z is not updated.
     *
     * @param z_new   New desired Z from coordinator.
     * @param des_z   Current des_z. Updated to z_new only if floor is released.
     * @return        True if the new goal was accepted (floor released).
     */
    bool notifyNewGoal(double z_new, double& des_z);

    void   reset();
    double filteredForce() const { return Fz_filt_; }
    double zFloor()        const { return z_floor_;  }

    static constexpr double DEADBAND    = 0.3;
    static constexpr double F_DES       = 1.0;
    static constexpr double F_EMERGENCY = 4.0;
    static constexpr double DZ_EMERGENCY= 0.001;
    static constexpr double DZ_STEP_MAX = 0.003;   // Max rise per cycle (m) = 0.375 m/s max
    static constexpr double DT          = 1.0 / 125.0;

private:
    double Fz_filt_;
    double alpha_rise_;   // Fast alpha when force is increasing
    double alpha_fall_;   // Slow alpha when force is decreasing
    double z_floor_;      // Current Z floor. -inf = inactive.
};