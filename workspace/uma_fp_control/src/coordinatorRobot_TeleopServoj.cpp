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
#include "std_msgs/Bool.h"
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
  coordinator(const std::string& robot_name){
    // Subscripciones a los topics
    //haptic_sub_ = nh_.subscribe("/haptic_topic", 1000, &coordinator::cb_readPose, this);
    haptic_sub_ = nh_.subscribe("/phantom_rojo/phantom/pose", 1000, &coordinator::cb_readPose, this); //"/AZUL/phantom/pose"
    hapticBt_sub_ = nh_.subscribe("/phantom_rojo/phantom/button/white", 1000, &coordinator::cb_readButton, this); //"/AZUL/phantom/button/white"
    pose_r1_sub_ = nh_.subscribe(robot_name + "/pose_topic", 1000, &coordinator::cmd_pose1, this);
    //pose_r2_sub_ = nh_.subscribe(robot_names[0] + "/pose_topic", 1000, &coordinator::cmd_pose2, this);
    //pub
    goal_pub_ = nh_.advertise<geometry_msgs::Point>(robot_name+"/coordinator/goal_position", 1000);
    /*for (const auto& robot_name : robot_names) {    
      goal_pubs_.push_back(nh_.advertise<geometry_msgs::Point>(robot_name + "/coordinator/goal_position", 1000));
    }*/

    stitch_.x =0.4871;
    stitch_.y = 0.13958;
    stitch_.z = -0.16655;
    T1_ = Eigen::MatrixXd::Identity(4, 4);

    // Inicialización de vectores
    white_button_.resize(2, 0);            // para 2 botones: azul y rojo
    phantomPoseRef_.resize(3, 0.0);        // x, y, z
    phantomPoseRefRojo_.resize(3, 0.0);    // x, y, z
  }
  //callbacks
  void cb_readButton(const std_msgs::Int8::ConstPtr& msg){
    white_button_[0] = msg->data;
  }
  void cb_readPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    //ROS_INFO_STREAM("-----entro cb_readPose----");
    if (white_button_[0] == 1){
      //METER TRANSFORMADA HAPTIC ROBOT
      //std::cout << "white_button_[0]= " << white_button_[0] << std::endl;
      phantomPose_.x = posT1_[0] + (msg->pose.position.x - phantomPoseRef_[0]);
      phantomPose_.y = posT1_[1] + (msg->pose.position.y - phantomPoseRef_[1]);
      phantomPose_.z = posT1_[2] + (msg->pose.position.z - phantomPoseRef_[2]);
      //composeHapticDesPose(phantomPose_);
      //goal_pubs_[0]
      goal_pub_.publish(phantomPose_);
    }
    else{
      phantomPoseRef_ = {msg->pose.position.x,msg->pose.position.y,msg->pose.position.z};
      phantomPose_.x = 0;
      phantomPose_.y = 0;
      phantomPose_.z = 0;
      //goal_pubs_[0]
      //goal_pub_.publish(phantomPose_);
      posT1_ = {T1_(0,3), T1_(1,3), T1_(2,3)};
    }
  }

  

  void cmd_pose1(const std_msgs::Float64MultiArray::ConstPtr& msg){
    //ROS_INFO_STREAM("-----entro cmd_pose1----");
    // Crear una matriz Eigen::MatrixXd para almacenar los datos del mensaje
    Eigen::MatrixXd matrix(4, 4);
    // Copiar los datos del mensaje a la matriz Eigen
    int index = 0;
    for (int j = 0; j < 4; ++j) {
        for (int i = 0; i < 4; ++i) {
            matrix(i,j) = msg->data[index++];
        }
    }
    T1_ = matrix;
    //ROS_INFO_STREAM("T1_= " << T1_);
  }

private:
  ros::NodeHandle nh_;
  //sub
  ros::Subscriber haptic_sub_;
  ros::Subscriber hapticBt_sub_;
  ros::Subscriber haptic_sub_rojo_;
  ros::Subscriber hapticBt_sub_rojo_;
  ros::Subscriber camera_sub_;
  ros::Subscriber ontology_sub_;
  ros::Subscriber num_sub_;
  ros::Subscriber type_sub_;
  ros::Subscriber pose_r1_sub_;
  ros::Subscriber pose_r2_sub_;
  //pub
  //std::vector<ros::Publisher> goal_pubs_;
  ros::Publisher goal_pub_;
  //variables
  geometry_msgs::Point stitch_;
  geometry_msgs::Point phantomPose_;
  std::vector<double> phantomPoseRef_;
  std::vector<double> posT1_;
  geometry_msgs::Point phantomPoseRojo_;
  std::vector<double>  phantomPoseRefRojo_;
  Eigen::MatrixXd T1_;
  Eigen::MatrixXd T2_;
  Eigen::MatrixXd dest_;
  Eigen::Matrix4d Tdesp_ = Eigen::Matrix4d::Zero();
  int grey_button_;
  std::vector<int> white_button_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "coordinator_node");
    ros::NodeHandle nh;

    std::string robot_name;
    nh.param<std::string>("robot_name", robot_name, "");  // default "auto" o el que uses

    coordinator coord(robot_name);
    ros::spin();
    return 0;
}