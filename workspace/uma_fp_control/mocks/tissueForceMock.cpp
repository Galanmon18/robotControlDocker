/**
 * @file tissueForceMock.cpp
 * @brief Virtual elastic contact model of instrument tip–tissue interaction.
 *
 * Physical model:
 *   The tissue surface is modelled as a horizontal plane at z = z_surface.
 *   When the TTP penetrates below this plane, a compressive axial force
 *   proportional to the penetration depth is generated (unilateral spring):
 *
 *     penetration = max(0, z_surface - TTP.z)
 *     F_z = K_tissue * penetration       (compression, positive = into tissue)
 *
 *   No lateral tissue forces are modelled (the instrument tip is assumed to
 *   move along its axis; lateral stiffness can be added as an extension).
 *
 *   The force is expressed in the robot base frame (world Z axis), which
 *   matches the convention expected by the hybridControl tissue force callback.
 *
 * Inputs (subscribed):
 *   /alice_pose_topic — T_TTP as 16-element Float64MultiArray (column-major 4x4)
 *
 * Output (published):
 *   /auto/MyhybridControl/tissue_force — Float64MultiArray [Fx, Fy, Fz] (N)
 *
 * ROS parameters (all have defaults):
 *   ~K_tissue    Spring stiffness (N/m). Default: 200.0
 *                Soft tissue: 50–150 N/m; stiffer tissue: 200–500 N/m
 *   ~z_surface   Z height of the tissue surface in world frame (m). Default: -0.065
 *   ~damping     Viscous damping coefficient (N·s/m). Default: 2.0
 *                Adds velocity-dependent term to reduce oscillation in mock.
 *   ~prefix      Robot namespace prefix. Default: "alice"
 */

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

// ---- State ---------------------------------------------------------------
static double   tip_z      = 0.0;   // Current TTP Z (world frame)
static double   tip_z_prev = 0.0;   // Previous TTP Z (for velocity estimate)
static bool     tip_ready  = false;

// ---- Parameters (set in main) --------------------------------------------
static double K_tissue  = 200.0;  // Spring stiffness (N/m)
static double z_surface = -0.05; // Tissue surface Z in world frame (m)
static double damping   =   2.0;  // Damping coefficient (N·s/m)

static constexpr double DT       = 1.0 / 125.0;  // Control period (s)
static constexpr double F_MAX    = 10.0;           // Hard safety clamp (N)

// --------------------------------------------------------------------------
// Helper: extract translation from column-major 4x4 flat array
// Column-major layout: translation at indices 12 (x), 13 (y), 14 (z)
// --------------------------------------------------------------------------
Eigen::Vector3d extractTranslation(const std::vector<double>& mat){
    return Eigen::Vector3d(mat[12], mat[13], mat[14]);
}

// --------------------------------------------------------------------------
// Callback
// --------------------------------------------------------------------------
void cb_ttp(const std_msgs::Float64MultiArray::ConstPtr& msg){
    if (msg->data.size() < 16) return;
    tip_z_prev = tip_z;
    tip_z      = extractTranslation(msg->data)(2);
    tip_ready  = true;
}

// --------------------------------------------------------------------------
int main(int argc, char **argv){
    ros::init(argc, argv, "tissueForceMock");
    ros::NodeHandle nh;
    ros::NodeHandle nh_param("~");

    std::string prefix;
    nh_param.param<double>("K_tissue",  K_tissue,   200.0);
    nh_param.param<double>("z_surface", z_surface, -0.05);
    nh_param.param<double>("damping",   damping,     2.0);
    nh_param.param<std::string>("prefix", prefix,  "auto");

    ros::Subscriber sub_ttp = nh.subscribe(
        "/" + prefix + "/pose_topic", 10, cb_ttp);

    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>(
        "/tissue_force", 10);

    ros::Rate rate(125);
    std_msgs::Float64MultiArray force_msg;
    force_msg.data.resize(3, 0.0);

    ROS_INFO("tissueForceMock started | K=%.1f N/m | z_surface=%.4f m | damping=%.1f",
             K_tissue, z_surface, damping);

    while (ros::ok()){
        ros::spinOnce();

        if (tip_ready) {
            const double penetration = std::max(0.0, z_surface - tip_z);

            if (penetration > 1e-5) {
                // Spring term: proportional to penetration depth
                const double F_spring = K_tissue * penetration;

                // Damping term: opposes the penetration velocity
                // tip_z decreasing (going deeper) → positive damping force (opposing)
                const double v_z    = (tip_z - tip_z_prev) / DT;
                const double F_damp = -damping * v_z;  // Opposes downward motion

                double Fz = F_spring + F_damp;

                // Safety clamp: never exceed F_MAX
                Fz = std::min(Fz, F_MAX);

                force_msg.data[0] = 0.0;
                force_msg.data[1] = 0.0;
                force_msg.data[2] = Fz;  // Positive = compression (tip into tissue)

                ROS_DEBUG_THROTTLE(0.5,
                    "Tissue mock | penetration=%.2f mm | F_spring=%.2f N | F_damp=%.2f N | Fz=%.2f N",
                    penetration*1000, F_spring, F_damp, Fz);
            } else {
                // No contact
                force_msg.data = {0.0, 0.0, 0.0};
            }
        } else {
            force_msg.data = {0.0, 0.0, 0.0};
        }

        pub.publish(force_msg);
        rate.sleep();
    }
    return 0;
}
