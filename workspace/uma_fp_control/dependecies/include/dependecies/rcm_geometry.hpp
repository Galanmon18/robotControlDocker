#pragma once

#include <Eigen/Dense>
#include <geometry_msgs/Point.h>

/**
 * @file rcm_geometry.hpp
 * @brief Pure geometric computation of the desired instrument pose for RCM control.
 *
 * No ROS node, no mutable state, no external dependencies beyond Eigen.
 * All methods are static — this class is a namespace with type safety.
 * Can be unit-tested independently of the ROS infrastructure.
 */
class RCMGeometry {
public:

    /**
     * @brief Computes the desired 4x4 homogeneous transform for the instrument.
     *
     * Builds an orthonormal basis {vX, vY, vZ} such that vZ points from the
     * fulcrum toward the desired TTP position. This encodes the RCM constraint
     * geometrically: the instrument axis always passes through the trocar.
     *
     *   vZ = normalise(desPose - fulcrum)        (instrument axis)
     *   vX = normalise(vZ × e_y)                 (perpendicular in world XZ plane)
     *   vY = vZ × vX                              (completes right-handed basis)
     *
     * The resulting rotation matrix R = [vX | vY | vZ] is placed at desPose.
     *
     * @param fulcrum  Fulcrum (trocar) position in world frame.
     * @param desPose  Desired TTP position in world frame.
     * @return         4x4 homogeneous transform: RCM-consistent orientation + desPose translation.
     *
     * @warning If desPose == fulcrum the norm is zero and the result is undefined.
     */
    static Eigen::Matrix4d computeDesiredPose(const geometry_msgs::Point& fulcrum,
                                              const geometry_msgs::Point& desPose);

private:
    // Pure translation matrix (no external UMA_trans dependency needed)
    static Eigen::Matrix4d translationMatrix(double x, double y, double z);
};
