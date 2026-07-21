#pragma once

#include <vector>
#include <array>

/**
 * @file abdomen_force_controller.hpp
 * @brief P/PI controller that converts abdominal contact forces into
 *        cartesian velocity corrections ("flee from force").
 *
 * Implements the upper feedback loop of the hybrid control diagram:
 *
 *   e_F  = F_d,abdomen - f_abdomen       (F_d = 0: desired contact = none)
 *   X_F  = K_abdomen * e_F               (force error → position displacement)
 *   v_F  = K_P * X_F                     (position displacement → velocity)
 *
 * Collapsed into one gain: v_F = K * (0 - f_abdomen) = -K * f_abdomen
 *
 * The output [vx, vy, vz] is added to the velocity from the position
 * controller before being sent to ur->speedl(). The instrument moves
 * away from whatever surface is pushing on the abdomen sensor.
 *
 * Only linear velocity is generated (no angular correction from force).
 * No ROS dependency. Stateful only if integral action is enabled.
 */
class AbdomenForceController {
public:
    /**
     * @param Kp            Proportional gain (m/s per N).
     * @param Ki            Integral gain (m/s² per N). Set to 0 for pure P.
     * @param dt            Control period (s). Must match the main loop rate.
     * @param threshold     Force magnitude below which no correction is applied (N).
     *                      Acts as deadband to suppress sensor noise at rest.
     * @param v_max         Maximum correction velocity magnitude per axis (m/s).
     */
    explicit AbdomenForceController(double Kp        = 0.04,
                                    double Ki        = 0.0,
                                    double dt        = 1.0/125.0,
                                    double threshold = 0.8,
                                    double v_max     = 0.05);

    /**
     * @brief Compute the cartesian velocity correction from abdominal forces.
     *
     * @param f_abdomen  Measured forces [Fx, Fy, Fz] at the effector-abdomen
     *                   interface, in robot base frame (N).
     * @return           Velocity correction [vx, vy, vz] (m/s).
     *                   Add this to the position controller output before speedl().
     */
    std::vector<double> compute(const std::vector<double>& f_abdomen);

    /** @brief Resets integrator state. Call when re-entering contact from free motion. */
    void reset();

    /** @brief True if any axis force exceeds the threshold this cycle. */
    bool isActive() const { return active_; }

private:
    double Kp_;
    double Ki_;
    double dt_;
    double threshold_;
    double v_max_;
    bool   active_;

    std::array<double, 3> integral_;  // Per-axis integral state
};
