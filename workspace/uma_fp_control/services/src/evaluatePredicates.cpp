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
#include "omni_msgs/OmniButtonEvent.h"
#include "geometry_msgs/PoseStamped.h" 
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
class SurgeonDataRecorder {
public:
  SurgeonDataRecorder(const std::vector<std::string>& robot_names): T1_(4, 4), T2_(4, 4) { 
    // Subscripciones a los topics
    pose_r1_sub_ = nh_.subscribe(robot_names[0] + "/pose_topic", 1000, &SurgeonDataRecorder::cmd_pose1, this);
    pose_r2_sub_ = nh_.subscribe(robot_names[1] + "/pose_topic", 1000, &SurgeonDataRecorder::cmd_pose2, this);
    in_sub_ = nh_.subscribe("/stitch/in", 1, &SurgeonDataRecorder::cmd_stitchInCallback, this);
    out_sub_ = nh_.subscribe("/stitch/out", 1000, &SurgeonDataRecorder::cmd_stitchOutCallback, this);
    rest_sub_ = nh_.subscribe("/stitch/rest", 1000, &SurgeonDataRecorder::cmd_stitchRestCallback, this);
    gripperR_sub_ = nh_.subscribe("/gripperR_topic", 1000, &SurgeonDataRecorder::cmd_rightGripperCallback, this);
    gripperL_sub_  = nh_.subscribe("/gripperL_topic", 1000, &SurgeonDataRecorder::cmd_leftGripperCallback, this);
    vision_sub_ = nh_.subscribe("/vision_topic", 1000, &SurgeonDataRecorder::cmd_visionCallback, this);
    //pub
    result_pub_ = nh_.advertise<std_msgs::UInt8MultiArray>("cmd_DPassistant", 1000);
  }
  //callbacksss
  //
  void cmd_pose1(const std_msgs::Float64MultiArray::ConstPtr& msg){
    Eigen::MatrixXd matrix(4, 4);
    int index = 0;
    for (int j = 0; j < 4; ++j) {
        for (int i = 0; i < 4; ++i) {
            matrix(i,j) = msg->data[index++];
        }
    }
    T1_ = matrix;
    checkAndPublishResult();
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
    checkAndPublishResult();
  }
  //
  void cmd_stitchInCallback(const geometry_msgs::Point::ConstPtr& msg)
  {
    stitch_in_.x = msg->x;
    stitch_in_.y = msg->y;
    stitch_in_.z = msg->z;
    checkAndPublishResult();
  }
  void cmd_stitchOutCallback(const geometry_msgs::Point::ConstPtr& msg)
  {
    stitch_out_.x = msg->x;
    stitch_out_.y = msg->y;
    stitch_out_.z = msg->z;
    checkAndPublishResult();
  }
  void cmd_stitchRestCallback(const geometry_msgs::Point::ConstPtr& msg)
  {
    stitch_rest_.x = msg->x;
    stitch_rest_.y = msg->y;
    stitch_rest_.z = msg->z;
    checkAndPublishResult();
  }
  //
  void cmd_rightGripperCallback(const std_msgs::Bool::ConstPtr& msg)
  {
    closeGripperR_= msg->data;
    checkAndPublishResult();
  }
  void cmd_leftGripperCallback(const std_msgs::Bool::ConstPtr& msg)
  {
    closeGripperL_ = msg->data;
    checkAndPublishResult();
  }
  void cmd_visionCallback(const std_msgs::Bool::ConstPtr& msg)
  {
    free_ = msg->data;
    checkAndPublishResult();
  }
  // Verifica las condiciones y publica el resultado
  void checkAndPublishResult() {
    /****************/
    double dx = T1_(0,3) - stitch_in_.x;
    double dy = T1_(1,3) - stitch_in_.y;
    double dz = T1_(2,3) - stitch_in_.z;
    dist_in_ =std::sqrt(dx*dx + dy*dy + dz*dz);
    /****************/
    dx = T2_(0,3) - stitch_out_.x;
    dy = T2_(1,3) - stitch_out_.y;
    dz = T2_(2,3) - stitch_out_.z;
    dist_out_ =std::sqrt(dx*dx + dy*dy + dz*dz);
    /****************/
    dx = T1_(0,3) - stitch_rest_.x;
    dy = T1_(1,3) - stitch_rest_.y;
    dz = T1_(2,3) - stitch_rest_.z;
    distR_rest_ =std::sqrt(dx*dx + dy*dy + dz*dz);
    /****************/
    dx = T2_(0,3) - stitch_rest_.x;
    dy = T2_(1,3) - stitch_rest_.y;
    dz = T2_(2,3) - stitch_rest_.z;
    distL_rest_ =std::sqrt(dx*dx + dy*dy + dz*dz);
    /****************/
    inPositionSutureR_ = (std::abs(dist_in_) < umbral_);
    inPositionSutureL_ = (std::abs(dist_out_) < umbral_);
    inPositionChangeR_ = (std::abs(distR_rest_) < umbral_);
    inPositionChangeL_ = (std::abs(distL_rest_) < umbral_);
    std_msgs::UInt8MultiArray msg;
    msg.data = {inPositionSutureR_, inPositionSutureL_, inPositionChangeR_, inPositionChangeL_, closeGripperR_, closeGripperL_, free_, finished_};
    result_pub_.publish(msg);
  }
private:
  ros::NodeHandle nh_;
  ros::Subscriber pose_r1_sub_;
  ros::Subscriber pose_r2_sub_;
  ros::Subscriber in_sub_;
  ros::Subscriber out_sub_;
  ros::Subscriber rest_sub_;
  ros::Subscriber gripperR_sub_;
  ros::Subscriber gripperL_sub_;
  ros::Subscriber vision_sub_;
  ros::Publisher result_pub_;
  //predicados
  bool inPositionSutureR_=false;
  bool inPositionSutureL_=false;
  bool inPositionChangeR_=false;
  bool inPositionChangeL_=false;
  bool closeGripperR_=false;
  bool closeGripperL_=false;
  bool free_ = false;
  bool finished_= false;
  //variables
  Eigen::MatrixXd T1_;
  Eigen::MatrixXd T2_;
  geometry_msgs::Point stitch_in_;
  geometry_msgs::Point stitch_out_;
  geometry_msgs::Point stitch_rest_;
  double dist_in_;
  double dist_out_;
  double distR_rest_;
  double distL_rest_;
  double umbral_ = 0.005;
};
int main(int argc, char **argv){
  ros::init(argc, argv, "evaluatePredicate");
  // Lee los parámetros de ROS
  ros::NodeHandle nh;
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
  SurgeonDataRecorder node_predicates(robot_names);
  ros::spin();
  return 0;
}