#include "dependecies/rcm_geometry.hpp"
#include <cmath>

Eigen::Matrix4d RCMGeometry::computeDesiredPose(const geometry_msgs::Point& fulcrum,
                                                 const geometry_msgs::Point& desPose) {
    // Vector from fulcrum to desired TTP — defines the instrument axis
    Eigen::Vector3d axis(desPose.x - fulcrum.x,
                         desPose.y - fulcrum.y,
                         desPose.z - fulcrum.z);

    // vZ: unit vector along the instrument axis (fulcrum → TTP)
    Eigen::Vector3d vZ = axis.normalized();

    // vX: perpendicular to vZ in the world XZ plane.
    // Computed as vZ × e_y = (vZ.z, 0, -vZ.x), then normalised.
    // This choice is arbitrary but deterministic for all non-degenerate orientations.
    Eigen::Vector3d vX(vZ(2), 0.0, -vZ(0));
    vX.normalize();

    // vY: completes the right-handed orthonormal basis
    Eigen::Vector3d vY = vZ.cross(vX);  // Already unit if vZ and vX are unit and orthogonal
    vY.normalize();

    // Desired rotation matrix: columns = [vX | vY | vZ]
    Eigen::Matrix4d W_Rdest = Eigen::Matrix4d::Identity();
    W_Rdest.block<3,1>(0,0) = vX;
    W_Rdest.block<3,1>(0,1) = vY;
    W_Rdest.block<3,1>(0,2) = vZ;

    // Pre-multiply by fulcrum translation (anchors the rotation around the trocar),
    // then override translation column with the actual desired position.
    Eigen::Matrix4d T_dest = translationMatrix(fulcrum.x, fulcrum.y, fulcrum.z) * W_Rdest;
    T_dest(0,3) = desPose.x;
    T_dest(1,3) = desPose.y;
    T_dest(2,3) = desPose.z;

    return T_dest;
}

Eigen::Matrix4d RCMGeometry::translationMatrix(double x, double y, double z) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0,3) = x;
    T(1,3) = y;
    T(2,3) = z;
    return T;
}
