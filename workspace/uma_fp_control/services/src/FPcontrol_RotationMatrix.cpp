/**
 * @file hybridControl.cpp
 * @brief Hybrid position+force controller for a laparoscopic surgical instrument.
 *
 * This class is a ROS orchestrator. It owns pub/sub infrastructure, reads
 * sensor state, and delegates computation to:
 *   - RCMGeometry         : desired pose construction (stateless, no ROS)
 *   - TipForceController  : tip force PI with EWA filter (stateful, no ROS)
 *
 * Control logic each cycle (computeRobotCinematic):
 *
 *   BRANCH A — normal operation (|F_abdomen| < 1 N on all axes):
 *     Hybrid coupling: RCM position control and tissue force modulation are
 *     computed together to produce a SINGLE velocity vector sent to the robot
 *     via ur->speedl(). The tissue force PI only modulates vz; position
 *     control handles vx, vy, wx, wy. They are not independent controllers.
 *
 *   BRANCH B — abdominal safety (|F_abdomen| >= 1 N on any axis):
 *     The fulcrum estimate is corrected with the current kinematics.
 *     A proportional correction velocity is published for monitoring.
 *     ur->speedl() is NOT called — robot holds its last command.
 *
 *   BRANCH C — waiting:
 *     No desired pose received yet. Robot holds position.
 *
 * Known issues (flagged with ⚠️):
 *   - The empty-prefix branch in frame name resolution looks inverted.
 *   - In Branch B, Fy is forced to zero — verify for all contact directions.
 *   - readTransform() returns an uninitialised matrix on TF failure.
 */

#include <services/FPcontrol_RotationMatrix.hpp>
#include "dependecies/rcm_geometry.hpp"
#include "dependecies/tip_force_controller.hpp"

// =============================================================================
// CONSTRUCTOR / DESTRUCTOR
// =============================================================================

hybridControl::hybridControl(ur_script* urScript, UMA_trans* umaTf,
                             ErrorPose* composeError,
                             PIDController* composePID, selectTool* selectTool,
                             FTSensor* ftSensor,
                             tf::TransformListener* tf_listener, std::string t_prefix)
    : ur(urScript), tr(umaTf),  controlPosition(composePID),
      tool(selectTool), ftSensor(ftSensor), listener(tf_listener),
      tipForceCtrl_(0.1), rcmGeometry_()
{
    std::cout << t_prefix << " control build" << std::endl;
    prefix_in       = t_prefix;
    desPoseReceived = false;
    newFulcrum      = false;

    // ---- Abdomen source configuration (read from ROS param server) ----
    // abdomen_source: "sensor" | "topic" | "none"
    //   "sensor" — reads directly from UDP F/T sensor (abdomen_sensor_ip required)
    //   "topic"  — subscribes to /abdomen_force (mock or external node)
    //   "none"   — forceAbdomen stays zero (no abdomen feedback, e.g. robot 2)
    ros::NodeHandle nh_param("~");
    nh_param.param<std::string>("abdomen_source",     abdomen_source_,     "none");
    nh_param.param<std::string>("abdomen_sensor_ip",  abdomen_sensor_ip_,  "192.168.1.1");

    if (abdomen_source_ == "sensor") {
        try {
            abdomenSensor_ = std::make_unique<FTSensor>(abdomen_sensor_ip_);
            ROS_INFO("Abdomen source: UDP sensor at %s", abdomen_sensor_ip_.c_str());
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to connect to abdomen sensor at %s: %s — falling back to none",
                      abdomen_sensor_ip_.c_str(), e.what());
            abdomen_source_ = "none";
        }
    } else if (abdomen_source_ == "topic") {
        abdomen_effector_force_sub_ = nh_.subscribe("/abdomen_force", 1000,
                                                    &hybridControl::cb_abdomenForceCallback, this);
        ROS_INFO("Abdomen source: topic /abdomen_force");
    } else {
        ROS_INFO("Abdomen source: none — forceAbdomen will remain zero");
    }

    // ---- Other subscribers ----
    goal_pos_sub_ = nh_.subscribe("/"+prefix_in+"/coordinator/goal_position", 1000,
                                  &hybridControl::cb_stitchCallback, this);
    ROS_INFO("-------------Subscribed to: %s", goal_pos_sub_.getTopic().c_str());
    tissue_force_sub_ = nh_.subscribe("/tissue_force", 1000,
                                      &hybridControl::cb_tissueForceCallback, this);

    // ---- Publishers (monitoring — actual robot motion goes via ur->speedl) ----
    ttp_pub_     = nh_.advertise<std_msgs::Float64MultiArray>("pose_topic",          1000);
    te_pub_      = nh_.advertise<std_msgs::Float64MultiArray>("effectorFinal_topic", 1000);
    vel_pub_     = nh_.advertise<std_msgs::Float64MultiArray>("velocity_topic",      1000);
    fulcrum_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("fulcrum",             1000);

    // ---- Sensor state initialisation ----
    desPose.x = 0.; desPose.y = 0.; desPose.z = 0.;
    forceAbdomen = {0.0, 0.0, 0.0};
    forceTissue  = {0.0, 0.0, 0.0};
    F_           = {0.0, 0.0, 0.0};
    P0[0] = 0.0; P0[1] = 0.0; P0[2] = 0.0;
}

hybridControl::~hybridControl(){
    std::cout << "Leaving gently hybridControl..." << std::endl;
}


// =============================================================================
// CALLBACKS
// =============================================================================

void hybridControl::cb_stitchCallback(const geometry_msgs::Point::ConstPtr& msg){
    // Let the force controller decide if descent is safe
    std::cout << "des pose " << std::endl;
    bool accepted = tipForceCtrl_.notifyNewGoal(msg->z, desPose.z);
    desPose.x = msg->x;
    desPose.y = msg->y;
    if (!accepted) {
        std::cout << "Descent blocked: tissue force too high" << std::endl;
    } else{
        desPose.z = msg->z;
    }
    desPoseReceived = true;
}

void hybridControl::cb_abdomenForceCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    forceAbdomen[0] = msg->data[0];
    forceAbdomen[1] = msg->data[1];
    forceAbdomen[2] = msg->data[2];
    rho_abd         = msg->data[3];
    //std::cout << "--cb_stitchCallback received" << std::endl;
}

void hybridControl::cb_tissueForceCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    forceTissue[0] = msg->data[0];
    forceTissue[1] = msg->data[1];
    forceTissue[2] = msg->data[2];
}


// =============================================================================
// HELPERS: SENSING
// =============================================================================

/**
 * @brief Reads T_E (world→effector) and T_TTP (world→TTP) from the ROS TF tree.
 * ❗ Empty-prefix branch looks inverted — verify.
 */
void hybridControl::updateTransforms(){
    if (prefix_in.empty()) {
        base_name    = prefix_in + "base";
        tool_name    = prefix_in + "tool0_controller";
        efector_name = prefix_in + "tool0";
    } else {
        base_name    = prefix_in + "_base";
        tool_name    = prefix_in + "_tool0_controller";
        efector_name = prefix_in + "_tool0";
    }
    T_E   = readTransform(base_name, efector_name);
    T_TTP = readTransform(base_name, tool_name);
}

/**
 * @brief Returns true if abdominal forces are within the safe range (<1 N all axes).
 */
bool hybridControl::isAbdomenSafe() const {
    return (std::abs(forceAbdomen[0]) < 1) &&
           (std::abs(forceAbdomen[1]) < 1) &&
           (std::abs(forceAbdomen[2]) < 2);
}

// =============================================================================
// HELPERS: CONTROL
// =============================================================================

/**
 * @brief Hybrid position + tissue force control pipeline.
 *
 * Implements the coupled scheme from the block diagram:
 *
 *   Tissue force loop (admittance):
 *     e_F = F_d,tissue - f_tissue
 *     ΔZ_des = K_tissue * e_F          ← position correction, NOT velocity
 *
 *   Position loop (RCM + PID):
 *     P_des = X_des + ΔZ_des           ← corrected desired pose
 *     T_dest = RCMGeometry(fulcrum, P_des)
 *     vel = PID(error(T_dest, T_TTP))  ← single velocity output
 *
 * There is ONE velocity generator (the PID). The tissue force controller only
 * moves the setpoint — it does not produce a separate velocity command.
 * This eliminates the coupling problem of the previous PI-velocity approach.
 *
 * @return Velocity [vx, vy, vz, wx, wy, 0] for ur->speedl().
 */
std::vector<double> hybridControl::runHybridControl(){
    // Step 1: tip force control
    tipForceCtrl_.applyLimit(forceTissue[2], desPose.z, 0.001);
    // Step 2: RCM geometry — desired pose with corrected Z and fulcrum constraint
    Eigen::Matrix4d T_dest = rcmGeometry_.computeDesiredPose(fulcrum_position, desPose);
    // Step 3: Cartesian PID — single velocity output from one controller
    auto diffPose  = error->computeErrorTf(T_dest, T_TTP);
    auto velVector = controlPosition->calculate(diffPose);
    Eigen::Matrix<double,4,1> velAngular;
    velAngular << velVector[3], velVector[4], velVector[5], 0.0;

    // ❗ Negated wx and zeroed wz — verify sign convention for this robot configuration
    return { velVector[0], velVector[1], velVector[2],
             -velAngular(0), velAngular(1), 0.0 };
}

/**
 * @brief Updates the fulcrum estimate every cycle.
 *
 * Two update sources are combined here, in priority order:
 *
 *   1. Coordinator callback (newFulcrum flag): a new fulcrum position has been
 *      published externally (e.g. from a vision or geometry estimator). This is
 *      the primary source and is consumed once per message.
 *
 *   2. Abdominal force correction: when |F_abdomen| > 1 N on any axis, the robot
 *      is in contact with the trocar region and the instrument has moved relative
 *      to the initial fulcrum estimate. The fulcrum is recomputed from the current
 *      kinematics (T_E * E_T_Fp) to reflect the actual instrument position.
 *      This runs every cycle as long as forces are present — it is NOT blocking.
 *
 * By running this before the position controller, the hybrid control loop always
 * uses the most up-to-date fulcrum estimate regardless of contact state.
 */
void hybridControl::updateFulcrumState(){
    if (newFulcrum) {
        fulcrum_position = computeFulcrum(E_T_Fp, T_E);
        newFulcrum = false;
    }
    if (!isAbdomenSafe()) {
        // Abdominal contact detected: recompute fulcrum from current kinematics. Displacement from previous cycle acording to the force in the abdomen
        //step 1: make P_f_ coincident with I. rho_abd is the distance from the {6} to the abdomen
        //std::cout << "NO SAFE ABDOMEN DETECTED: " <<forceAbdomen[0] << ", " << forceAbdomen[1] << ", " << forceAbdomen[2] << std::endl;
        double K_adbomen = 0.005;
        Eigen::MatrixXd E_T_Fp_corrected = tr->desp({0,0,rho_abd});
        fulcrum_position = computeFulcrum(E_T_Fp_corrected, T_E);
        //setp 2: correct horizontal displacement acording to force
        fulcrum_position.x += 1*K_adbomen*base_forceAbdomen[0]; 
        fulcrum_position.y += 1*K_adbomen*base_forceAbdomen[1];
        std::cout << "....................................." << std::endl;
        std::cout << "forceAbdomen=[" << base_forceAbdomen[0] << ", " << base_forceAbdomen[1] << ", " << base_forceAbdomen[2] << "]" << std::endl;
        std::cout << "fulcrum_position=[" << fulcrum_position.x << ", " << fulcrum_position.y << ", " << fulcrum_position.z << "]" << std::endl;
    }
}


// =============================================================================
// HELPERS: PUBLISHING
// =============================================================================

void hybridControl::publishVelocity(const std::vector<double>& vel){
    array_vel_.data = vel;
    vel_pub_.publish(array_vel_);
    array_vel_.data.clear();
}

/**
 * @brief Publishes T_TTP, T_E (column-major flat arrays) and fulcrum position.
 */
void hybridControl::publishState(){
    fulcrum_position_.data = { fulcrum_position.x, fulcrum_position.y, fulcrum_position.z };
    fulcrum_pub_.publish(fulcrum_position_);
    fulcrum_position_.data.clear();

    for (int i = 0; i < T_TTP.rows(); ++i)
        for (int j = 0; j < T_TTP.cols(); ++j)
            pose_.data.push_back(T_TTP(j,i));
    ttp_pub_.publish(pose_);
    pose_.data.clear();

    for (int i = 0; i < T_E.rows(); ++i)
        for (int j = 0; j < T_E.cols(); ++j)
            poseE_.data.push_back(T_E(j,i));
    te_pub_.publish(poseE_);
    poseE_.data.clear();
}


// =============================================================================
// LOW-LEVEL HELPERS
// =============================================================================

Eigen::MatrixXd hybridControl::readTransform(std::string base, std::string tool0){
    Eigen::MatrixXd T(4,4);
    try {
        listener->waitForTransform(base, tool0, ros::Time(0), ros::Duration(1.0));
        listener->lookupTransform(base, tool0, ros::Time(0), tf_pose);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR_STREAM("TF Exception: " << ex.what());
        return T;  // ❗ Uninitialised on failure
    }
    const double qX = tf_pose.getRotation().x();
    const double qY = tf_pose.getRotation().y();
    const double qZ = tf_pose.getRotation().z();
    const double qW = tf_pose.getRotation().w();
    const double X  = tf_pose.getOrigin().x();
    const double Y  = tf_pose.getOrigin().y();
    const double Z  = tf_pose.getOrigin().z();
    T << (1-2*(qY*qY+qZ*qZ)), (2*(qX*qY-qW*qZ)),  2*(qX*qZ+qW*qY), X,
         (2*(qX*qY+qW*qZ)), (1-2*(qX*qX+qZ*qZ)),  2*(qY*qZ-qW*qX), Y,
         2*(qX*qZ-qW*qY),   2*(qY*qZ+qW*qX), (1-2*(qX*qX+qY*qY)), Z,
         0,                  0,                0,                    1;
    return T;
}


geometry_msgs::Point hybridControl::computeFulcrum(Eigen::MatrixXd E_T_Fp, Eigen::MatrixXd T_E){
    Eigen::MatrixXd T = T_E * E_T_Fp;
    geometry_msgs::Point p;
    p.x = T(0,3); p.y = T(1,3); p.z = T(2,3);
    return p;
}


// =============================================================================
// INITIALISATION
// =============================================================================

void hybridControl::initializeRobot(int type, double p_estimado_init, double tool_length, bool move_to_init,
                                    std::vector<double> initPosition){
    std::cout << "Initialising robot RM..." << std::endl;
    if (move_to_init) {
        ur->set_tcp(tool0);
        ros::Duration(0.5).sleep();
        //ur.set_tcp(tcp);
        ROS_INFO_STREAM("Guardando la posicion del TCP");
        ros::Duration(0.5).sleep();
        // Movimiento del robot hacia un punto cómodo.
        ROS_INFO_STREAM("moviendo al punto");
        ur->movej(initPosition, true);
        ros::Duration(0.5).sleep();
        ur->stopl(1); 
    }
    std::vector<double> tcp;
    std::vector<double> dfp;
    int length_axis;
    ros::param::get("/" + prefix_in + "/tool/tcp", tcp);
    ros::param::get("/" + prefix_in + "/tool/dfp", dfp);
    ros::param::get("/" + prefix_in + "/tool/length_axis",length_axis);
    tcp[length_axis] = tool_length;
    dfp[length_axis] = p_estimado_init;
    Eigen::MatrixXd rot       = tr->rotZ(tcp[5]) * tr->rotY(tcp[4]) * tr->rotX(tcp[3]);
    //E_T_TTP = tr->desp({tcp[0], tcp[1], tcp[2]}) * rot;
    E_T_Fp  = tr->desp({dfp[0], dfp[1], dfp[2]}) * rot;
    TCP     = tcp;
    //tool->computeTwrist(type, p_estimado_init, tool_length, tr);
    //E_T_Fp = tool->E_T_Fp;
    //TCP    = tool->TCP;
    std::cout << "TCP: " << TCP[0] << ", " << TCP[1] << ", " << TCP[2]  << ", " << TCP[3] << ", " << TCP[4] << ", " << TCP[5] << std::endl;
    ur->set_tcp(TCP);
    ros::Duration(1).sleep();
    newFulcrum = true;
    if (!abdomenSensor_) {
        std::cerr << "abdomenSensor_ is NULL\n";
    } else {
        std::cout << "abdomenSensor_ OK\n";
        if (abdomenSensor_->tareSensor()) {
            std::cout << "Sensor tared successfully\n";
            ros::Duration(0.2).sleep();
        } else {
            std::cerr << "Failed to tare sensor\n";
        }
    }
}


// =============================================================================
// MAIN CONTROL LOOP
// =============================================================================

/**
 * @brief Main control callback. Called at 125 Hz.
 *
 * Execution order each cycle:
 *
 *   1. updateTransforms()    — read T_E and T_TTP from TF.
 *   2. updateFulcrumState()  — update fulcrum from coordinator OR from abdominal
 *                              force correction. Both sources run every cycle:
 *                              they are NOT mutually exclusive. If abdominal
 *                              forces are present, the corrected fulcrum is
 *                              written to fulcrum_position_ before step 3.
 *   3. runHybridControl()    — position (RCM) + tissue force PI, always using
 *                              the fulcrum from step 2 (already corrected if
 *                              abdominal forces were detected this cycle).
 *   4. ur->speedl()          — send the coupled velocity to the robot.
 *   5. publishVelocity/State — monitoring.
 *
 * The abdominal force correction is no longer a blocking branch: it feeds
 * the position controller in the same cycle rather than replacing it.
 */
void hybridControl::computeRobotCinematic(double /*L*/){
    // Step 1: Read current robot pose from TF
    updateTransforms();

    // Step 2: Read abdominal forces according to the configured source.
    // "sensor": blocking UDP read — matches 125 Hz loop, sensor configured at 500 Hz.
    // "topic":  forceAbdomen already updated by cb_abdomenForceCallback.
    // "none":   forceAbdomen stays {0,0,0}.
    if (abdomen_source_ == "sensor" && abdomenSensor_) {
        std::vector<double> rawAbdomen(6);
        if (abdomenSensor_->readFT(rawAbdomen)) {
            forceAbdomen[0] = rawAbdomen[0];
            forceAbdomen[1] = rawAbdomen[1];
            forceAbdomen[2] = rawAbdomen[2];
        } else {
            ROS_WARN_THROTTLE(1.0, "Abdomen sensor read failed — using last known value");
        }
    }
 
    // Transform abdomen forces from sensor frame to robot base frame
    Eigen::Vector3d f_base = T_E.topLeftCorner<3,3>() *
                             Eigen::Vector3d(forceAbdomen[0],
                                             forceAbdomen[1],
                                             forceAbdomen[2]);
    base_forceAbdomen = {f_base(0), f_base(1), f_base(2)};
    updateFulcrumState();
    //desPoseReceived = true;
    if (!desPoseReceived) {
        publishState();
        return;
    }
    // Step 3+4: Hybrid control (position + tissue force) with the latest fulcrum,
    // which already incorporates any abdominal force correction from this cycle.
    auto vel = runHybridControl();
    ur->speedl(vel, 0.1, 0.1);

    // Step 5: Monitoring
    publishVelocity(vel);
    publishState();
}


// =============================================================================
// MAIN
// =============================================================================

int main(int argc, char **argv){
    ros::init(argc, argv, "MyhybridControl");
    ros::NodeHandle nh_param("~");
    ros::Rate rate(125);  // 125 Hz, matches servoj dt = 8 ms

    std::string prefix, sensor_ip;
    double kp, p_estimado_init, tool_length;
    double base, shoulder, elbow, wrist1, wrist2, wrist3;
    int type;
    bool move_to_init;

    nh_param.param<std::string>("prefix",      prefix,          "alice");
    nh_param.param<double>     ("kp",          kp,              1.0);
    nh_param.param<double>     ("p_estimado",  p_estimado_init, 0.1);
    nh_param.param<double>     ("tool_length", tool_length,     0.2);
    nh_param.param<int>        ("type",        type,            3);
    nh_param.param<std::string>("sensor_ip",   sensor_ip,       "192.168.1.1");
    nh_param.param<bool>       ("move_to_init",move_to_init,    false);
    // abdomen_source and abdomen_sensor_ip are read inside the constructor
    // via the private NodeHandle (~). Set them in the launch file:
    //   <param name="abdomen_source"    value="sensor"/>  <!-- or "topic" or "none" -->
    //   <param name="abdomen_sensor_ip" value="192.168.1.2"/>
    nh_param.param<double>     ("base",        base,            90.0);
    nh_param.param<double>     ("shoulder",    shoulder,        90.0);
    nh_param.param<double>     ("elbow",       elbow,           90.0);
    nh_param.param<double>     ("wrist1",      wrist1,          90.0);
    nh_param.param<double>     ("wrist2",      wrist2,          90.0);
    nh_param.param<double>     ("wrist3",      wrist3,           0.0);

    std::vector<double> initPosition = {
        base     * DEG_TO_RAD,
        shoulder * DEG_TO_RAD,
        elbow    * DEG_TO_RAD,
        wrist1   * DEG_TO_RAD,
        wrist2   * DEG_TO_RAD,
        wrist3   * DEG_TO_RAD
    };

    ur_script             ur(prefix);
    UMA_trans             tr;
    PIDController         controlPosition(kp, 0, 0, 0.008, 6);
    selectTool            tool;
    ErrorPose             error;
    tf::TransformListener tf_listener;
    FTSensor              ftSensor(sensor_ip);

    hybridControl robot(&ur, &tr,  &error, &controlPosition, &tool,
                        &ftSensor, &tf_listener, prefix);
    robot.initializeRobot(type, p_estimado_init, tool_length, move_to_init, initPosition);
    ros::Duration(1.5).sleep();

    robot.computeRobotCinematic(tool_length);
    std::cout << "Entering hybrid control loop at 125 Hz" << std::endl;

    while (ros::ok()){
        robot.computeRobotCinematic(tool_length);
        ros::spinOnce();
        rate.sleep();
    }
}