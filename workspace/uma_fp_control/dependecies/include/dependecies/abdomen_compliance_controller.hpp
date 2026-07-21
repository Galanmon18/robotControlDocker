#pragma once

#include <Eigen/Dense>
#include <array>

/**
 * @file abdomen_compliance_controller.hpp
 * @brief Compliance-based orientation controller for abdominal force rejection.
 *
 * Implements the upper loop of the decoupled hybrid controller:
 *
 *   θx_des = Fx / (K_abd * (L - ρ))
 *   θy_des = Fy / (K_abd * (L - ρ))
 *   e_θ    = θ_des - θ_current
 *   [wx,wy] = Kp * e_θ + Ki * ∫e_θ
 *
 * Physical interpretation:
 *   The instrument moves linearly toward the goal. When the trocar physically
 *   prevents it (generating abdominal forces), this controller converts those
 *   forces into desired tilt angles. The RCM constraint emerges from the
 *   compliance rather than being imposed geometrically.
 *
 *   Lever arm (L - ρ):
 *     L   = total instrument length (m)
 *     ρ   = external length, trocar plane → flange (m), from abdomenForceMock
 *     L-ρ = internal length = mechanical advantage at the fulcrum
 *     Larger internal length → less tilt per Newton (more leverage)
 *
 * No ROS dependency. Stateful only for the angular PI integrators.
 */
class AbdomenComplianceController {
public:
    /**
     * @param K_abd    Compliance gain (N/rad). Larger = stiffer, less tilt per N.
     * @param Kp       Angular PI proportional gain (rad/s per rad).
     * @param Ki       Angular PI integral gain (rad/s² per rad).
     */
    explicit AbdomenComplianceController(double K_abd = 50.0,
                                          double Kp    = 1.0,
                                          double Ki    = 0.1);

    /**
     * @brief Compute angular velocity correction [wx, wy] from abdominal forces.
     *
     * @param Fx        Abdominal force in X (N).
     * @param Fy        Abdominal force in Y (N).
     * @param rho       External instrument length ρ (m).
     * @param L_total   Total instrument length (m).
     * @param R_current Current TCP rotation matrix (3x3), from T_TTP.
     * @return          [wx, wy] angular velocity (rad/s).
     */
    std::array<double,2> compute(double Fx, double Fy,
                                  double rho, double L_total);

    /** @brief Resets integrator state. */
    void reset();

    double desiredThetaX() const { return theta_x_des_; }
    double desiredThetaY() const { return theta_y_des_; }
    double currentThetaX() const { return theta_x_cur_; }
    double currentThetaY() const { return theta_y_cur_; }

private:
    double K_abd_;
    double Kp_;
    double Ki_;

    // PI integrator state
    double theta_x_int_;
    double theta_y_int_;

    // Cached for diagnostics
    double theta_x_des_;
    double theta_y_des_;
    double theta_x_cur_;
    double theta_y_cur_;

    static constexpr double THETA_MAX      = 0.35;   // ~20° max tilt (rad)
    //static constexpr double INT_MAX        = 0.3;    // Integrator saturation (rad·s)
    static constexpr double L_MIN_INTERNAL = 0.02;   // Minimum lever arm (m)
    static constexpr double DT             = 1.0 / 125.0;

    /**
     * Compliance law: θ = F / (K_abd * (L - ρ))
     * Clamps result to [-THETA_MAX, THETA_MAX].
     */
    void computeDesiredAngles(double Fx, double Fy,
                               double L_total, double rho);

    /**
     * Extract current tilt angles from TCP rotation matrix.
     * Uses small-angle approximation (valid for |θ| < ~20°):
     *   θx ≈  R[2,1]   (Z-axis tilt in Y direction)
     *   θy ≈ -R[2,0]   (Z-axis tilt in -X direction)
     */
    void extractCurrentAngles(const Eigen::Matrix3d& R);
};
