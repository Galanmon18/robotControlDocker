/**
 * @file abdomenForceMock.cpp
 * @brief Virtual elastic model of the instrument–abdominal wall interaction,
 *        with configurable fulcrum perturbation for experiments.
 *
 * Physical model (unchanged):
 *   F_abdomen = -K_abd * (P_cross.xy - P_fulcrum_effective.xy)
 *   d_external = |P_eff - P_cross|
 *
 * Perturbation system:
 *   P_fulcrum_effective = P_fulcrum_nominal + P_perturbation
 *
 *   P_perturbation can come from two sources (priority order):
 *     1. External topic /fulcrum_perturbation [dx, dy, dz] (m):
 *        A separate node publishes the desired fulcrum displacement.
 *        Useful for complex/manual perturbations.
 *        If no message received in the last 0.5 s, falls back to source 2.
 *
 *     2. Built-in signal generator (controlled by ROS params):
 *        Generates sine, step, or ramp perturbations autonomously.
 *        Use this for repeatable experiments without a separate node.
 *
 * ROS parameters:
 *   ~K_abd              Spring stiffness (N/m). Default: 50.0
 *   ~prefix             Robot namespace prefix. Default: "alice"
 *
 *   Perturbation signal (built-in generator):
 *   ~perturb_type       "none" | "sine" | "step" | "ramp". Default: "none"
 *   ~perturb_axis       "x" | "y" | "xy". Default: "x"
 *   ~perturb_amplitude  Signal amplitude (m). Default: 0.005 (5 mm)
 *   ~perturb_freq       Sine frequency (Hz). Default: 0.5
 *   ~perturb_step_time  Time (s) after which step activates. Default: 3.0
 *
 * Topics:
 *   Sub: /<prefix>/pose_topic          — T_TTP (16-elem column-major 4x4)
 *   Sub: /<prefix>/effectorFinal_topic — T_E   (16-elem column-major 4x4)
 *   Sub: /<prefix>/fulcrum             — [x,y,z] nominal fulcrum (3-elem)
 *   Sub: /fulcrum_perturbation         — [dx,dy,dz] external perturbation (3-elem, optional)
 *   Pub: /abdomen_force                — [Fx, Fy, 0, d_external]
 *   Pub: /fulcrum_effective            — [x,y,z] effective fulcrum (monitoring)
 */

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <Eigen/Dense>
#include <cmath>
#include <string>

// ---- Robot state ----------------------------------------------------------
static Eigen::Vector3d P_tip     = Eigen::Vector3d::Zero();
static Eigen::Vector3d P_eff     = Eigen::Vector3d::Zero();
static Eigen::Vector3d P_fulcrum = Eigen::Vector3d::Zero();  // Nominal fulcrum

static bool tip_ready     = false;
static bool eff_ready     = false;
static bool fulcrum_ready = false;

// ---- External perturbation state -----------------------------------------
static Eigen::Vector3d P_ext_perturb = Eigen::Vector3d::Zero();
static ros::Time       last_ext_time;
static bool            ext_perturb_received = false;
static const double    EXT_TIMEOUT = 0.5;  // Seconds before falling back to built-in

// ---- Parameters ----------------------------------------------------------
static double      K_abd            = 50.0;
static std::string perturb_type     = "none";   // "none"|"sine"|"step"|"ramp"
static std::string perturb_axis     = "x";      // "x"|"y"|"xy"
static double      perturb_amplitude= 0.005;    // m
static double      perturb_freq     = 0.5;      // Hz (sine only)
static double      perturb_step_time= 3.0;      // s (step only)


// ==========================================================================
// Built-in signal generator
// ==========================================================================

/**
 * @brief Generate a perturbation vector from the built-in signal generator.
 *
 * @param t   Time elapsed since node start (s).
 * @return    [dx, dy, dz] perturbation in metres.
 *
 * Signal types:
 *   "none" → always zero.
 *   "sine" → A * sin(2π*f*t) on the selected axis. Simulates rhythmic motion
 *             like breathing (0.25–0.5 Hz, 3–8 mm amplitude).
 *   "step" → 0 until t = step_time, then A. Simulates sudden patient movement.
 *   "ramp" → linearly grows from 0 to A over step_time seconds, then holds A.
 *             Simulates slow drift.
 */
Eigen::Vector3d generateBuiltinPerturbation(double t){
    double signal = 0.0;

    if (perturb_type == "sine") {
        signal = perturb_amplitude * std::sin(2.0 * M_PI * perturb_freq * t);
    }
    else if (perturb_type == "step") {
        signal = (t >= perturb_step_time) ? perturb_amplitude : 0.0;
    }
    else if (perturb_type == "ramp") {
        signal = (t < perturb_step_time)
                     ? perturb_amplitude * (t / perturb_step_time)
                     : perturb_amplitude;
    }
    // "none" → signal stays 0

    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    if      (perturb_axis == "x")  { p(0) = signal; }
    else if (perturb_axis == "y")  { p(1) = signal; }
    else if (perturb_axis == "xy") { p(0) = signal; p(1) = signal; }

    return p;
}


// ==========================================================================
// Helpers
// ==========================================================================

/**
 * Extract translation from column-major 4x4 flat array.
 * Translation column (col 3) at indices 12 (x), 13 (y), 14 (z).
 */
Eigen::Vector3d extractTranslation(const std::vector<double>& data){
    return Eigen::Vector3d(data[12], data[13], data[14]);
}

/**
 * Find point on line (P1→P2) at z = z_target.
 * P(t) = P1 + t*(P2-P1), solve t s.t. P(t).z = z_target.
 */
Eigen::Vector3d lineAtZ(const Eigen::Vector3d& P1,
                         const Eigen::Vector3d& P2,
                         double z_target){
    const double dz = P2(2) - P1(2);
    if (std::abs(dz) < 1e-9) return P1;
    const double t = (z_target - P1(2)) / dz;
    return P1 + t * (P2 - P1);
}


// ==========================================================================
// Callbacks
// ==========================================================================

void cb_ttp(const std_msgs::Float64MultiArray::ConstPtr& msg){
    if (msg->data.size() < 16) return;
    P_tip     = extractTranslation(msg->data);
    tip_ready = true;
}

void cb_effector(const std_msgs::Float64MultiArray::ConstPtr& msg){
    if (msg->data.size() < 16) return;
    P_eff     = extractTranslation(msg->data);
    eff_ready = true;
}

void cb_fulcrum(const std_msgs::Float64MultiArray::ConstPtr& msg){
    if (msg->data.size() < 3) return;
    /*P_fulcrum(0)  = msg->data[0];//  ← uncomment when topic is reliable
    P_fulcrum(1)  = msg->data[1];//
    P_fulcrum(2)  = msg->data[2];*/
    P_fulcrum(0)  = 0.1073 ;//0.13104;  
    P_fulcrum(1)  = -0.299;//-0.2985;  
    P_fulcrum(2)  = 0.1627;// 0.15329;
    //[0.10734304725707601, -0.2996650314947336, 0.16270279977891328]

    fulcrum_ready = true;
}

/**
 * External perturbation topic: [dx, dy, dz] in metres.
 * Publish 0,0,0 to disable. Falls back to built-in if no message
 * arrives within EXT_TIMEOUT seconds.
 */
void cb_perturbation(const std_msgs::Float64MultiArray::ConstPtr& msg){
    if (msg->data.size() < 3) return;
    P_ext_perturb(0)    = msg->data[0];
    P_ext_perturb(1)    = msg->data[1];
    P_ext_perturb(2)    = msg->data[2];
    last_ext_time        = ros::Time::now();
    ext_perturb_received = true;
}


// ==========================================================================
// Main
// ==========================================================================

int main(int argc, char **argv){
    ros::init(argc, argv, "abdomenForceMock");
    ros::NodeHandle nh;
    ros::NodeHandle nh_param("~");

    // Load parameters
    std::string prefix;
    nh_param.param<double>     ("K_abd",              K_abd,             50.0);
    nh_param.param<std::string>("prefix",             prefix,            "auto");
    nh_param.param<std::string>("perturb_type",       perturb_type,      "none");
    nh_param.param<std::string>("perturb_axis",       perturb_axis,      "x");
    nh_param.param<double>     ("perturb_amplitude",  perturb_amplitude,  0.005);
    nh_param.param<double>     ("perturb_freq",       perturb_freq,       0.5);
    nh_param.param<double>     ("perturb_step_time",  perturb_step_time,  3.0);

    // Subscribers
    ros::Subscriber sub_ttp = nh.subscribe(
        "/" + prefix + "/pose_topic", 10, cb_ttp);
    ros::Subscriber sub_eff = nh.subscribe(
        "/" + prefix + "/effectorFinal_topic", 10, cb_effector);
    ros::Subscriber sub_fulcrum = nh.subscribe(
        "/" + prefix + "/fulcrum", 10, cb_fulcrum);
    ros::Subscriber sub_perturb = nh.subscribe(
        "/fulcrum_perturbation", 10, cb_perturbation);

    // Publishers
    ros::Publisher pub_force = nh.advertise<std_msgs::Float64MultiArray>(
        "/abdomen_force", 10);
    ros::Publisher pub_fulcrum_eff = nh.advertise<std_msgs::Float64MultiArray>(
        "/fulcrum_effective", 10);  // Monitoring: where the fulcrum appears to be

    ros::Rate rate(125);

    std_msgs::Float64MultiArray force_msg;
    force_msg.data.resize(4, 0.0);

    std_msgs::Float64MultiArray fulcrum_eff_msg;
    fulcrum_eff_msg.data.resize(3, 0.0);

    const ros::Time t_start = ros::Time::now();

    ROS_INFO("abdomenForceMock | K=%.1f N/m | perturb_type=%s | axis=%s | A=%.1f mm | prefix=%s",
             K_abd, perturb_type.c_str(), perturb_axis.c_str(),
             perturb_amplitude * 1000.0, prefix.c_str());

    while (ros::ok()){
        ros::spinOnce();

        // ---- Compute effective fulcrum = nominal + perturbation ----
        // Select perturbation source: external topic if fresh, else built-in
        Eigen::Vector3d perturbation;
        if (ext_perturb_received &&
            (ros::Time::now() - last_ext_time).toSec() < EXT_TIMEOUT) {
            // External topic is active and fresh
            perturbation = P_ext_perturb;
        } else {
            // Built-in generator
            const double t = (ros::Time::now() - t_start).toSec();
            perturbation   = generateBuiltinPerturbation(t);
        }

        const Eigen::Vector3d P_fulcrum_eff = P_fulcrum + perturbation;

        // ---- Publish effective fulcrum for monitoring ----
        fulcrum_eff_msg.data = { P_fulcrum_eff(0), P_fulcrum_eff(1), P_fulcrum_eff(2) };
        pub_fulcrum_eff.publish(fulcrum_eff_msg);

        // ---- Compute forces ----
        if (tip_ready && eff_ready && fulcrum_ready) {
            const Eigen::Vector3d P_cross = lineAtZ(P_tip, P_eff, P_fulcrum_eff(2));

            // Lateral deviation from EFFECTIVE fulcrum (includes perturbation)
            const double dx = P_cross(0) - P_fulcrum_eff(0);
            const double dy = P_cross(1) - P_fulcrum_eff(1);

            force_msg.data[0] = -K_abd * dx;
            force_msg.data[1] = -K_abd * dy;
            force_msg.data[2] =  0.0;
            force_msg.data[3] = (P_eff - P_cross).norm();  // d_external (m)

            ROS_DEBUG_THROTTLE(1.0,
                "Abdomen mock | perturb=(%.2f,%.2f) mm | dev=(%.2f,%.2f) mm | F=(%.2f,%.2f) N",
                perturbation(0)*1000, perturbation(1)*1000,
                dx*1000, dy*1000,
                force_msg.data[0], force_msg.data[1]);
        } else {
            force_msg.data = {0.0, 0.0, 0.0, 0.0};
        }

        pub_force.publish(force_msg);
        rate.sleep();
    }
    return 0;
}