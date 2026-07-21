//Coordinador 2 robots moviendose al mismo punto que le manda la camara. 
//En este código de experimentacion las coordenadas son las mismas para cada robot, algo que está mal porque no respeta el sistema de coordenadas global
//pero como tampoco uso la camara de verdad, solamente me invento puntos, no pasa nada
//----------INPUTS-------------//
//  haptics
//  camera
//  ontology
//  nº robot
//  robot type
//  robot pose (e.g. ontology will use)
//----------OUTPUTS------------//
//  des position or des displacement
//  tool action (close, open, ...)
//  initial pose
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string.h>
#include "std_msgs/UInt8MultiArray.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include <cmath>
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <Eigen/Dense>
//#include "omni_msgs/OmniButtonEvent.h"
#include "geometry_msgs/PoseStamped.h" 
#include "std_msgs/Int8.h"

class coordinator {
public:
  coordinator(const std::vector<std::string>& robot_names){
    // Subscripciones a los topics
    haptic_sub_ = nh_.subscribe("/haptic_topic", 1000, &coordinator::cb_readPose, this); //haptic_sub_ = nh_.subscribe("/AZUL/phantom/pose", 1000, &coordinator::cb_readPose, this);
    hapticBt_sub_ = nh_.subscribe("/AZUL/phantom/button/white", 1000, &coordinator::cb_readButton, this);
    camera_sub_ = nh_.subscribe("cmd", 1000, &coordinator::cmd_camera, this);
    ontology_sub_ = nh_.subscribe("ontology_topic", 1000, &coordinator::cmd_ontology, this);
    pose_r1_sub_ = nh_.subscribe(robot_names[0] + "/pose_topic", 1000, &coordinator::cmd_pose1, this);
    pose_r2_sub_ = nh_.subscribe(robot_names[1] + "/pose_topic", 1000, &coordinator::cmd_pose2, this);
    //pub
    for (const auto& robot_name : robot_names) {    
      goal_pubs_.push_back(nh_.advertise<geometry_msgs::Point>(robot_name + "/coordinator/goal_position", 1000));
    }
  }
  void composeHapticDesPose(geometry_msgs::Point msg) {
    Tdesp_ << 1, 0, 0, msg.x,
              0, 1, 0, msg.y,
              0, 0, 1, msg.z,
              0, 0, 0, 1;
    dest_ = Tdesp_ * T1_;
    stitch_.x = dest_(0,3);
    stitch_.y = dest_(1,3);
    stitch_.z = dest_(2,3);
    goal_pubs_[0].publish(stitch_);
  }
  //callbacks
  void cb_readButton(const std_msgs::Int8::ConstPtr& msg){
    white_button_ = msg->data;
  }
  //other read pose but now the data type is geometry_msgs::Point
  void cb_readPose(const geometry_msgs::Point::ConstPtr& msg){
    //ROS_INFO_STREAM("-----entro cmd_haptic----");
    phantomPose_.x = msg->x;
    phantomPose_.y = msg->y;
    phantomPose_.z = msg->z;
    goal_pubs_[0].publish(phantomPose_);
    //composeHapticDesPose(phantomPose_);
  }
  void cmd_camera(const std_msgs::UInt8MultiArray::ConstPtr& msg){
    //ROS_INFO_STREAM("-----entro cmd_camera----");
    stitch_.x = 0.333;
    stitch_.y = -0.3;
    stitch_.z = 0.3;
    goal_pubs_[1].publish(stitch_);
  }
  void cmd_ontology(const geometry_msgs::Pose::ConstPtr& msg){
    ROS_INFO_STREAM("-----entro cmd_ontology----");
  }
  void cmd_numRobot(const std_msgs::Float64::ConstPtr& msg){
    ROS_INFO_STREAM("-----entro cmd_numRobot----");
  }
  void cmd_type(const std_msgs::Float64::ConstPtr& msg){
    ROS_INFO_STREAM("-----entro cmd_type----");
  }
  void cmd_pose1(const std_msgs::Float64MultiArray::ConstPtr& msg){
    Eigen::MatrixXd matrix(4, 4);
    int index = 0;
    for (int j = 0; j < 4; ++j) {
        for (int i = 0; i < 4; ++i) {
            matrix(i,j) = msg->data[index++];
        }
    }
    T1_ = matrix;
  }
  void cmd_pose2(const std_msgs::Float64MultiArray::ConstPtr& msg){
    Eigen::MatrixXd matrix(4, 4);
    int index = 0;
    for (int j = 0; j < 4; ++j) {
        for (int i = 0; i < 4; ++i) {
            matrix(i,j) = msg->data[index++];
        }
    }
    T2_ = matrix;
  }
private:
  ros::NodeHandle nh_;
  //sub
  ros::Subscriber haptic_sub_;
  ros::Subscriber hapticBt_sub_;
  ros::Subscriber camera_sub_;
  ros::Subscriber ontology_sub_;
  ros::Subscriber num_sub_;
  ros::Subscriber type_sub_;
  ros::Subscriber pose_r1_sub_;
  ros::Subscriber pose_r2_sub_;
  //pub
  std::vector<ros::Publisher> goal_pubs_;
  //variables
  geometry_msgs::Point stitch_;
  geometry_msgs::Point phantomPose_;
  std::vector<double> phantomPoseRef_;
  Eigen::MatrixXd T1_;
  Eigen::MatrixXd T2_;
  Eigen::MatrixXd dest_;
  Eigen::Matrix4d Tdesp_ = Eigen::Matrix4d::Zero();
  int grey_button_;
  int white_button_;
};

int main(int argc, char** argv) {
  // Inicializa el nodo ROS
  ros::init(argc, argv, "coordinator_node");
  // Crea un manejador de nodo ROS
  ros::NodeHandle nh;
  // Lee los parámetros de ROS
  std::string robot_names_str;
  nh.param<std::string>("robot_names", robot_names_str, "alice,bob");
  // Elimina los espacios en blanco de la cadena de nombres de los robots
  robot_names_str.erase(std::remove_if(robot_names_str.begin(), robot_names_str.end(), ::isspace), robot_names_str.end());
  // Extrae los nombres de los robots del string
  std::vector<std::string> robot_names;
  std::stringstream ss(robot_names_str);
  std::string token;
  while (std::getline(ss, token, ',')) {
    robot_names.push_back(token);
  }
  // Crea una instancia de la clase coordinator con los parámetros obtenidos
  coordinator coord(robot_names);
  // Gira el bucle ROS
  ros::spin();
  return 0;
}