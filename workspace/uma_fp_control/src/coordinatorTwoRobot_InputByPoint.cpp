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

class coordinator {
public:
  coordinator(const std::vector<std::string>& robot_names) {
    // Subscripciones a los topics
    haptic_sub_ = nh_.subscribe("haptic_topic", 1000, &coordinator::cmd_haptic, this);
    camera_sub_ = nh_.subscribe("cmd_key", 1000, &coordinator::cmd_camera, this);
    ontology_sub_ = nh_.subscribe("ontology_topic", 1000, &coordinator::cmd_ontology, this);
    pose_sub_ = nh_.subscribe("pose_topic", 1000, &coordinator::cmd_pose, this);
    //pub
    for (const auto& robot_name : robot_names) {    
      // Publicador para cada robot
      goal_pubs_.push_back(nh_.advertise<geometry_msgs::Point>(robot_name + "/coordinator/goal_position", 1000));
    }
  }
  //callbacks
  void cmd_haptic(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("-----entro cmd_haptic----");
  }
  void cmd_camera(const std_msgs::UInt8MultiArray::ConstPtr& msg){
    ROS_INFO_STREAM("-----entro cmd_camera----");
    stitch_.x = 0.225;
    stitch_.y = -0.075;
    stitch_.z = 0.307;
    goal_pubs_[0].publish(stitch_);
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
  void cmd_pose(const geometry_msgs::Pose::ConstPtr& msg){
    ROS_INFO_STREAM("-----entro cmd_pose----");
  }
private:
  ros::NodeHandle nh_;
  //sub
  ros::Subscriber haptic_sub_;
  ros::Subscriber camera_sub_;
  ros::Subscriber ontology_sub_;
  ros::Subscriber num_sub_;
  ros::Subscriber type_sub_;
  ros::Subscriber pose_sub_;
  //pub
  std::vector<ros::Publisher> goal_pubs_;
  //variables
  geometry_msgs::Point stitch_;
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