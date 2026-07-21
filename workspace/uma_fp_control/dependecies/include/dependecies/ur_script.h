// Nombre: ur_script.h
// Paquete: craneeal_usage
// Libreria: ur_script_lib

// Descripción:
// Cabecera de la libreria de funciones UR Script

#ifndef UR_SCRIPT_HEADER
#define UR_SCRIPT_HEADER

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <sstream>
#include <vector>
#include <string>
#include <Eigen/Geometry>
#include <std_msgs/Int8.h>
//#include <craneeal_usage/key_teleop.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/Pose.h"

#define Q_CHECK_COMPLETION_TOL 0.01  // 0.01 rad
#define L_CHECK_COMPLETION_TOL 0.1e-3 // 0.1 mm

#define DEFAULT_Q_VEL 0.5 // rad/s
#define DEFAULT_Q_ACC 0.5 // rad/s2

#define DEFAULT_L_VEL 0.1 // m/s
#define DEFAULT_L_ACC 0.1 // m/s2

class ur_script
{
    ros::Publisher publisher;
    ros::NodeHandle node_handle;
    tf2_ros::TransformListener tf_listener;
    tf2_ros::Buffer tf_buffer;

    std::string joints_to_string(std::vector<double> vector);
    std::string pose_to_string(std::vector<double> vector);
    std::string geometry_pose_to_string(geometry_msgs::Pose pose);

    std::string qd_to_string(std::vector<double> qd);
    
    std::string get_inverse_kin(std::vector<double> vector);
    std::string offset_tool(std::vector<double> vector);
    std::string offset_tool_rel(std::vector<double> offset, std::vector<double> joints);

    bool check_completion_q(std::vector<double> goal);
    bool check_completion_pose(std::vector<double> goal);

    double teleop_step;

public:
    //ur_script();
    ur_script(const std::string& topic_prefix);

    void freedrive();
    void set_tcp(std::vector<double> xd);
    void movej(std::vector<double> q, bool blocking = false, double a = DEFAULT_Q_ACC, double v = DEFAULT_Q_VEL, double t = 0.0, double r = 0.0);
    void movej_pose(std::vector<double> pose, bool blocking = false, double a = DEFAULT_Q_ACC, double v = DEFAULT_Q_VEL, double t = 0.0, double r = 0.0);
    void movej_offset_tool(std::vector<double> offset_xyz, double a = DEFAULT_Q_ACC, double v = DEFAULT_Q_VEL, double t = 0.0, double r = 0.0);
    void movej_offset_tool_rel(std::vector<double> joints, std::vector<double> offset_xyz, double a = DEFAULT_Q_ACC, double v = DEFAULT_Q_VEL, double t = 0.0, double r = 0.0);
    
    void move_laparoscopy(geometry_msgs::Point posFinal, geometry_msgs::Point posRef, bool blocking = false, double a = DEFAULT_Q_ACC, double v = DEFAULT_Q_VEL, double t = 0.0, double r = 0.0);
    
    void movel(geometry_msgs::Pose pose, bool blocking = false, double a = DEFAULT_L_ACC, double v = DEFAULT_L_VEL, double t = 0.0, double r = 0.0);
    //void movel(std::vector<double> pose, bool blocking = false, double a = DEFAULT_L_ACC, double v = DEFAULT_L_VEL, double t = 0.0, double r = 0.0);
    void movel_offset_tool(std::vector<double> offset_xyz, double a = DEFAULT_L_ACC, double v = DEFAULT_L_VEL, double t = 0.0, double r = 0.0);
    void movel_offset_tool_rel(std::vector<double> joints, std::vector<double> offset_xyz, double a = DEFAULT_L_ACC, double v = DEFAULT_L_VEL, double t = 0.0, double r = 0.0);
    Eigen::Affine3d transformationMatix();
    void servoj(std::vector<double> q, double a = DEFAULT_Q_ACC, double v = DEFAULT_Q_VEL, double t = 0.008, double la_t = 0.1, double gain = 300.0);
    void servoj_ik(std::vector<double> pose, double a = DEFAULT_Q_ACC, double v = DEFAULT_Q_VEL, double t = 0.008, double la_t = 0.1, double gain = 300.0);
    void servoj_offset_tool_rel(std::vector<double> joints, std::vector<double> offset_xyz, double a = DEFAULT_Q_ACC, double v = DEFAULT_Q_VEL, double t = 0.008, double la_t = 0.1, double gain = 300.0);

    void speedl(std::vector<double> xd, double a, double t);

    void stopl(double a);

    void set_var();
    std::vector<double> quaternion_to_aa(geometry_msgs::Pose quaternion);
    void print_status();

    void teleop_callback(const std_msgs::Int8ConstPtr &msg);

    //variable
    std::string prefix="";
    std::string base_name;
    std::string tool_name;
};

#endif