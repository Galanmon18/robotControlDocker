#pragma once

// ROS core
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Point.h"
#include <tf/transform_listener.h>

// Eigen
#include <Eigen/Dense>
using Eigen::Vector3d;

// Project dependencies
#include <dependecies/computeError.hpp>
#include <dependecies/computePID.hpp>
#include <dependecies/computeT.hpp>
#include <dependecies/selectTool.hpp>
#include <dependecies/ur_script.h>
#include "dependecies/hex_ft_udp.hpp"
#include "dependecies/rcm_geometry.hpp"
#include "dependecies/tip_force_controller.hpp"
#include "dependecies/abdomen_force_controller.hpp"
#include "dependecies/abdomen_compliance_controller.hpp"

#define DEG_TO_RAD 0.017453293
#define RAD_TO_DEG 57.295779513

/**
 * @file hybridControl.hpp
 * @brief Hybrid position+force controller for a laparoscopic surgical instrument.
 *
 * Public API: constructor, initializeRobot(), computeRobotCinematic().
 * Everything else is private implementation detail.
 */
class hybridControl {

public:
    // -------------------------------------------------------------------------
    // Construction
    // -------------------------------------------------------------------------

    /**
     * @brief Wires all external dependencies, sets up ROS pub/sub, initialises state.
     * @param urScript    UR robot command interface.
     * @param umaTf       Homogeneous transform utilities.
     * @param initRobot   Robot homing logic.
     * @param composeError  Cartesian pose error computation.
     * @param composePID    Cartesian PID controller (pose error → velocity).
     * @param selectTool    Tool geometry and fixed-transform computation.
     * @param ftSensor      F/T sensor handle (forces arrive via ROS topic).
     * @param tf_listener   ROS TF tree listener.
     * @param t_prefix      Robot namespace prefix (e.g. "alice").
     */
    hybridControl(ur_script* urScript, UMA_trans* umaTf, ErrorPose* composeError, PIDController* composePID,
                  selectTool* selectTool, FTSensor* ftSensor,
                  tf::TransformListener* tf_listener, std::string t_prefix);

    ~hybridControl();

    // -------------------------------------------------------------------------
    // Public interface
    // -------------------------------------------------------------------------

    /**
     * @brief Moves the robot to the initial joint configuration and configures the tool.
     * @param type             Tool geometry type identifier (used by selectTool).
     * @param p_estimado       Initial estimate of effector-to-trocar distance (m).
     * @param tool_length      Total instrument length (m).
     * @param initPosition     Initial joint angles in radians [6].
     */
    void initializeRobot(int type, double p_estimado, double tool_length, bool move_to_init,
                         std::vector<double> initPosition);

    /**
     * @brief Main control callback. Must be called at 125 Hz from the main loop.
     *
     * Each cycle:
     *   1. Reads T_E and T_TTP from TF.
     *   2. Updates fulcrum estimate (coordinator callback + abdominal force correction).
     *   3. Computes hybrid velocity (RCM position + tissue force admittance).
     *   4. Sends velocity to robot via ur->speedl().
     *   5. Publishes state for monitoring.
     *
     * @param L  Tool length (reserved for future use).
     */
    void computeRobotCinematic(double L);

private:
    // -------------------------------------------------------------------------
    // External dependencies (injected via constructor, not owned)
    // -------------------------------------------------------------------------
    ur_script*             ur;
    UMA_trans*             tr;
    selectTool*            tool;
    PIDController*         controlPosition;
    FTSensor*              ftSensor;
    ErrorPose*             error;
    tf::TransformListener* listener;

    // -------------------------------------------------------------------------
    // Internal control modules (owned by this class)
    // -------------------------------------------------------------------------
    TipForceController tipForceCtrl_;
    RCMGeometry rcmGeometry_;
    AbdomenForceController abdomenForceCtrl_;
    AbdomenComplianceController abdomenComplianceCtrl_;
    //--------------------------------------------------------------------------
    // Internal sensors
    //--------------------------------------------------------------------------
    // Abdomen source configuration
    std::string                  abdomen_source_;      // "sensor" | "topic" | "none"
    std::string                  abdomen_sensor_ip_;
    std::unique_ptr<FTSensor>    abdomenSensor_;        // Only non-null if source="sensor"
    // -------------------------------------------------------------------------
    // ROS infrastructure
    // -------------------------------------------------------------------------
    ros::NodeHandle nh_;

    // Subscribers
    ros::Subscriber goal_pos_sub_;              // Desired TTP position (world frame)
    ros::Subscriber abdomen_effector_force_sub_;// Forces at effector-abdomen interface [Fx,Fy,Fz] (N)
    ros::Subscriber tissue_force_sub_;          // Forces at instrument tip against tissue [Fx,Fy,Fz] (N)

    // Publishers (monitoring only — robot motion is commanded via ur->speedl)
    ros::Publisher ttp_pub_;      // T_TTP as 4x4 column-major flat array
    ros::Publisher te_pub_;       // T_E   as 4x4 column-major flat array
    ros::Publisher vel_pub_;      // Commanded cartesian velocity [vx,vy,vz,wx,wy,wz]
    ros::Publisher fulcrum_pub_;  // Estimated fulcrum position [x,y,z]

    // Pre-allocated message buffers (avoid per-cycle allocation)
    std_msgs::Float64MultiArray pose_;             // T_TTP serialisation buffer
    std_msgs::Float64MultiArray poseE_;            // T_E   serialisation buffer
    std_msgs::Float64MultiArray array_vel_;        // Velocity message buffer
    std_msgs::Float64MultiArray fulcrum_position_; // Fulcrum message buffer

    // -------------------------------------------------------------------------
    // Robot configuration (set once during initializeRobot)
    // -------------------------------------------------------------------------
    std::vector<double> tool0 = {0., 0., 0., 0., 0., 0.}; // Zero tool offset (initial homing)
    std::vector<double> TCP;      // TCP transform sent to the UR controller
    Eigen::MatrixXd     E_T_Fp;   // Fixed transform: effector → fulcrum point (tool geometry)
    std::string         prefix_in;// Robot namespace prefix (e.g. "alice")

    // -------------------------------------------------------------------------
    // Kinematic state (updated every cycle by updateTransforms)
    // -------------------------------------------------------------------------
    Eigen::MatrixXd T_E;    // world → effector (flange), 4x4
    Eigen::MatrixXd T_TTP;  // world → TTP (tool tip controller frame), 4x4
    tf::StampedTransform tf_pose; // Raw TF result (reused across TF lookups)

    // TF frame name strings (built from prefix_in each cycle)
    std::string base_name;
    std::string tool_name;
    std::string efector_name;

    // -------------------------------------------------------------------------
    // Sensor state (written by callbacks, read by control methods)
    // -------------------------------------------------------------------------
    geometry_msgs::Point desPose;           // Desired TTP position in world frame
    bool                 desPoseReceived;   // True once the first goal has arrived

    geometry_msgs::Point fulcrum_position;  // Current fulcrum estimate in world frame
    bool                 newFulcrum;        // Flag: unprocessed fulcrum callback pending
    Vector3d             P0;               // Raw fulcrum coordinates from callback [x,y,z]

    std::vector<double> forceAbdomen;  // [Fx, Fy, Fz] at effector-abdomen interface (N)
    std::vector<double> base_forceAbdomen;// [Fx, Fy, Fz] at base-abdomen interface (N)
    std::vector<double> forceTissue;   // [Fx, Fy, Fz] at instrument tip (N)
    std::vector<double> F_;             // Force without decoupling (N)

    double rho_abd;           // External instrument length ρ (m), from abdomenForceMock
    double tool_length_;      // Stored from initializeRobot for use in compliance
    // -------------------------------------------------------------------------
    // Intermediate control state (written and read within computeRobotCinematic)
    // -------------------------------------------------------------------------
    std::vector<double> delta_force;  // Proportional abdominal force correction (m/s)

    // -------------------------------------------------------------------------
    // Callbacks
    // -------------------------------------------------------------------------
    void cb_stitchCallback(const geometry_msgs::Point::ConstPtr& msg);
    void cb_fulcrumCallback(const geometry_msgs::Point::ConstPtr& msg);
    void cb_abdomenForceCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void cb_tissueForceCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

    // -------------------------------------------------------------------------
    // Control pipeline methods (called in order by computeRobotCinematic)
    // -------------------------------------------------------------------------

    /** Reads T_E and T_TTP from the ROS TF tree. */
    void updateTransforms();

    /** Returns true if |F_abdomen| < 1 N on all axes (safe to move). */
    bool isAbdomenSafe() const;

    /**
     * Updates fulcrum_position each cycle from two non-exclusive sources:
     *   1. Coordinator callback (newFulcrum flag).
     *   2. Abdominal force correction: recomputes from current kinematics when
     *      |F_abdomen| > 1 N, so the position controller always uses a fresh estimate.
     */
    void updateFulcrumState();

    /**
     * Computes the hybrid control velocity:
     *   admittance (tissue force → ΔZ_des) → RCM pose → PID → single velocity output.
     */
    std::vector<double> runHybridControl();

    // -------------------------------------------------------------------------
    // Publishing helpers
    // -------------------------------------------------------------------------
    void publishVelocity(const std::vector<double>& vel);
    void publishState();

    // -------------------------------------------------------------------------
    // Low-level helpers
    // -------------------------------------------------------------------------

    /**
     * Reads T_base_tool from the TF tree and converts quaternion+translation to 4x4.
     * ❗ Returns an uninitialised matrix on TF failure — caller must guard.
     */
    Eigen::MatrixXd readTransform(std::string base, std::string tool0);

    /**
     * Proportional abdominal force controller: delta = Kf * F.
     * Used to compute monitoring velocity during abdominal contact.
     */
    std::vector<double> forceControl(const double& Kf, std::vector<double> forces);

    /**
     * Computes fulcrum position in world frame: T_fulcrum = T_E * E_T_Fp.
     */
    geometry_msgs::Point computeFulcrum(Eigen::MatrixXd E_T_Fp, Eigen::MatrixXd T_E);
};