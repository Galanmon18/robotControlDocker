/**
 * @file FPcontrol_jacobian.cpp
 * @brief Decoupled hybrid controller for laparoscopic instrument.
 *
 * Two independent loops (from block diagram):
 *
 *   UPPER LOOP — AbdomenComplianceController:
 *     Fx, Fy  → θx_des = Fx / (K_abd*(L-ρ))
 *               θy_des = Fy / (K_abd*(L-ρ))
 *     e_θ     → angular PI → [wx, wy]
 *
 *   LOWER LOOP — TipForceController + PID:
 *     forceTissue[2] → applyLimit() → desPose.z corrected
 *     T_dest (current rotation + corrected translation) → PID → [vx, vy, vz]
 *
 *   OUTPUT: ur->speedl([vx, vy, vz, wx, wy, 0])
 */

#include <services/FPcontrol_RotationMatrix.hpp>
#include "dependecies/tip_force_controller.hpp"
#include "dependecies/abdomen_compliance_controller.hpp"

// =============================================================================
// CONSTRUCTOR / DESTRUCTOR
// =============================================================================

hybridControl::hybridControl(ur_script* urScript, UMA_trans* umaTf,
                             ErrorPose* composeError,
                             PIDController* composePID, selectTool* selectTool,
                             FTSensor* ftSensor,
                             tf::TransformListener* tf_listener, std::string t_prefix)
    : ur(urScript), tr(umaTf), controlPosition(composePID),
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
    bool accepted = tipForceCtrl_.notifyNewGoal(msg->z, desPose.z);
    desPose.x = msg->x;
    desPose.y = msg->y;
    if (accepted) desPose.z = msg->z;
    else          std::cout << "Descent blocked: tissue force too high" << std::endl;
    desPoseReceived = true;
}

void hybridControl::cb_abdomenForceCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    forceAbdomen[0] = msg->data[0];
    forceAbdomen[1] = msg->data[1];
    forceAbdomen[2] = msg->data[2];
    rho_abd         = msg->data[3];  // ρ: external instrument length (m)
}

void hybridControl::cb_tissueForceCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    forceTissue[0] = msg->data[0];
    forceTissue[1] = msg->data[1];
    forceTissue[2] = msg->data[2];
}

// =============================================================================
// SENSING
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
    return (std::abs(forceAbdomen[0]) < 0.8) &&
           (std::abs(forceAbdomen[1]) < 0.8) &&
           (std::abs(forceAbdomen[2]) < 0.8);
}

void hybridControl::updateFulcrumState(){
    // Recompute for monitoring only — compliance strategy doesn't use fulcrum for control
    fulcrum_position = computeFulcrum(E_T_Fp, T_E);
}

// =============================================================================
// MAIN CONTROL PIPELINE
// =============================================================================

/**
 * @brief Decoupled hybrid control. Returns [vx, vy, vz, wx, wy, 0].
 *
 * LOWER LOOP (position + tissue force):
 *   1. TipForceController::applyLimit() raises desPose.z if Fz > F_DES.
 *   2. T_dest = current TCP rotation + corrected translation.
 *      Using current rotation decouples position from orientation:
 *      the position PID only sees translation error → angular output ≈ 0.
 *   3. PID → [vx, vy, vz].
 *
 * UPPER LOOP (orientation compliance):
 *   4. AbdomenComplianceController::compute():
 *      compliance law → desired angles → extract current angles → PI → [wx, wy]
 *
 * @param L_total  Total instrument length (m).
 */
std::vector<double> hybridControl::runHybridControl(){

    // --- LOWER LOOP ---

    // Step 1: tissue force limiter modifies desPose.z in-place
    tipForceCtrl_.applyLimit(forceTissue[2], desPose.z, 0.001);

    // Step 2: T_dest = current rotation + desired translation
    // This decouples the position PID from orientation: angular error = 0 by construction
    Eigen::Matrix4d T_dest = Eigen::Matrix4d::Identity();
    T_dest.block<3,3>(0,0) = T_TTP.block<3,3>(0,0).cast<double>();
    T_dest(0,3) = desPose.x;
    T_dest(1,3) = desPose.y;
    T_dest(2,3) = desPose.z;
    // Step 3: position PID → [vx, vy, vz, ~0, ~0, ~0]
    auto diffPose = error->computeErrorTf(T_dest, T_TTP);
    auto velPos   = controlPosition->calculate(diffPose);

    // --- UPPER LOOP ---

    // Step 4: AbdomenComplianceController handles everything:
    //   θ_des from compliance law, θ_cur from rotation matrix, PI → [wx, wy]
    auto ang = abdomenComplianceCtrl_.compute(forceAbdomen[0], forceAbdomen[1],
                                                rho_abd, tool_length_);
    if (isAbdomenSafe()) {
        ang = {0.0, 0.0};
    }
    //ang = {0.0, 0.0};
    return { velPos[0], velPos[1], velPos[2],
            ang[0],    ang[1],    0.0 };
}

// =============================================================================
// PUBLISHING
// =============================================================================

void hybridControl::publishVelocity(const std::vector<double>& vel){
    array_vel_.data = vel;
    vel_pub_.publish(array_vel_);
    array_vel_.data.clear();
}

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
        return T;
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

geometry_msgs::Point hybridControl::computeFulcrum(Eigen::MatrixXd E_T_Fp,
                                                    Eigen::MatrixXd T_E){
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
    std::cout << "Initialising robot C..." << std::endl;
    if (move_to_init) {
        ROS_INFO_STREAM("move_to_init: ");
        ROS_INFO_STREAM(move_to_init);
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
    ur->set_tcp(TCP);
    ros::Duration(0.5).sleep();
    newFulcrum = true;
    if (!abdomenSensor_) {
        std::cerr << "abdomenSensor_ is NULL\n";
    } else {
        std::cout << "abdomenSensor_ OK\n";
        if (abdomenSensor_->tareSensor()) {
            std::cout << "Sensor tared successfully\n";
        } else {
            std::cerr << "Failed to tare sensor\n";
        }
    }
}

// =============================================================================
// MAIN CONTROL LOOP
// =============================================================================

void hybridControl::computeRobotCinematic(double L){
    // Step 1: Read current robot pose from TF
    updateTransforms();
    updateFulcrumState();

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

    //Transform abdomen sensor data to robot base frame using T_E base_forceAbdomen = (T_E.topLeftCorner<3,3>()) * forceAbdomen;
    Eigen::Vector3d f_base = T_E.topLeftCorner<3,3>() *
                         Eigen::Vector3d(forceAbdomen[0],
                                         forceAbdomen[1],
                                         forceAbdomen[2]);
    base_forceAbdomen = {f_base(0), f_base(1), f_base(2)};

    if (!desPoseReceived) {
        publishState();
        return;
    }
    tool_length_ = L;
    auto vel = runHybridControl();
    ur->speedl(vel, 0.1, 0.1);
    publishVelocity(vel);
    publishState();
}

// =============================================================================
// MAIN
// =============================================================================

int main(int argc, char **argv){
    ros::init(argc, argv, "MyhybridControl");
    ros::NodeHandle nh_param("~");
    ros::Rate rate(125);

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
    nh_param.param<bool>       ("move_to_init",move_to_init,    true);
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
        base     * DEG_TO_RAD, shoulder * DEG_TO_RAD, elbow  * DEG_TO_RAD,
        wrist1   * DEG_TO_RAD, wrist2   * DEG_TO_RAD, wrist3 * DEG_TO_RAD
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
    std::cout << "Entering COMPLIANCE control loop at 125 Hz" << std::endl;
    std::cout << "prefix= " << prefix << "; move_to_init= " << move_to_init<< "; p_estimado= " << p_estimado_init<< std::endl;

    while (ros::ok()){
        robot.computeRobotCinematic(tool_length);
        ros::spinOnce();
        rate.sleep();
    }
}