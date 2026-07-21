// system coordinator

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
#include "geometry_msgs/PoseStamped.h" 

class coordinator {
public:
  coordinator(const std::string& robot_name) {
    // Subscripciones a los topics
    haptic_sub_ = nh_.subscribe("/haptic_topic", 1000, &coordinator::cmd_haptic, this);
    camera_sub_ = nh_.subscribe("/camera", 1000, &coordinator::cmd_camera, this);
    ontology_sub_ = nh_.subscribe("ontology_topic", 1000, &coordinator::cmd_ontology, this);
    num_sub_ = nh_.subscribe("num_robot_topic", 1000, &coordinator::cmd_numRobot, this);
    type_sub_ = nh_.subscribe("type_robot_topic", 1000, &coordinator::cmd_type, this);
    pose_sub_ = nh_.subscribe(robot_name+"/pose_topic", 1000, &coordinator::cmd_pose, this);
    //pub
    goal_pub_ = nh_.advertise<geometry_msgs::Point>(robot_name+"/coordinator/goal_position", 1000);
    // Inicializa la posición de la aguja
    stitch_.x = 0;
    stitch_.y = 0;
    stitch_.z = 0;
    goal_pub_.publish(stitch_);
  }
  //callbacks
  void cmd_haptic(const geometry_msgs::Point& msg){
    ROS_INFO_STREAM("-----entro cmd_haptic----");
    stitch_.x = T1_(0,3) + msg.x;
    stitch_.y = T1_(1,3) + msg.y;
    stitch_.z = T1_(2,3) + msg.z;
    goal_pub_.publish(stitch_);
    ROS_INFO_STREAM("-----salgo cmd_haptic----");
  }
  void cmd_camera(const std_msgs::UInt8MultiArray::ConstPtr& msg){
    ROS_INFO_STREAM("-----entro cmd_camera----");
    stitch_.x = 0.0;
    stitch_.y = 0.0;
    stitch_.z = 0.0;
    goal_pub_.publish(stitch_);
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
  void cmd_pose(const std_msgs::Float64MultiArray::ConstPtr& msg){
    Eigen::MatrixXd matrix(4, 4);
    int index = 0;
    for (int j = 0; j < 4; ++j) {
        for (int i = 0; i < 4; ++i) {
            matrix(i,j) = msg->data[index++];
        }
    }
    T1_ = matrix;
    first_ = true;
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
  ros::Publisher goal_pub_;
  ros::Publisher init_pub_;
  bool agujaIzquierda_=true;
  //variables
  Eigen::MatrixXd T1_;
  geometry_msgs::Point stitch_;
  std_msgs::Float64MultiArray initPosition_;
  double umbral_ = 0.015;
  bool first_ = false;
};

int main(int argc, char **argv){
  ros::init(argc, argv, "coordinator");
  ros::NodeHandle nh;
  std::string robot_name;
  nh.param<std::string>("prefix", robot_name, "auto");
  coordinator node_predicates(robot_name);
  ros::spin();
  return 0;
}