//Este codigo debe simular el funcionamiento de la ontologia. Para ello:)
//1. Publicar en ROS en numero de areas de sutura en el topic /num_areas
//2. Publicar en ROS el topic /potential_field/attraction/point
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
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include <cmath>
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <Eigen/Dense>
#include "omni_msgs/OmniButtonEvent.h"
#include "geometry_msgs/PoseStamped.h" 
#include "std_msgs/Int8.h"
#include "uma_fp_control/stitchArea.h"

class coordinator {
public:
  coordinator(const std::vector<std::string>& robot_names){
    //pub
    num_areas_pub_ = nh_.advertise<std_msgs::Int32>("num_areas", 1000);
    area_pub_ = nh_.advertise<uma_fp_control::stitchArea>("current_area", 1000);
    finished_pub_ = nh_.advertise<std_msgs::UInt8MultiArray>("finished", 1000);
    availableInsertion_pub_ = nh_.advertise<std_msgs::UInt8MultiArray>("availableInsertion", 1000);
    availableExtraction_pub_ = nh_.advertise<std_msgs::UInt8MultiArray>("availableExtraction", 1000);
    for (const auto& robot_name : robot_names) {    
      goal_pubs_.push_back(nh_.advertise<geometry_msgs::Point>(robot_name + "/coordinator/goal_position", 1000));
      goal_gripper_pubs_.push_back(nh_.advertise<std_msgs::Bool>(robot_name + "/gripper/state", 1000));
    }
  }
  void initE(){
    //matriz paso de 1 a 2
    Tpaso_ << -0.4481,0.8940,0,-0.4203,
              -0.8940,-0.4481,0,0.6807,
               0,0,1,0,
               0,0,0,1; //mirar dual_robot.xacro
    //reference frame 1
    area1_.insertion.x = 0.13105;
    area1_.insertion.y = -0.36246;
    area1_.insertion.z = -0.1033;
    area1_.extraction.x = -0.13105;
    area1_.extraction.y = -0.36246;
    area1_.extraction.z = -0.1033;
    //se supone que change es el mismo punto pero visto en dos sistemas de referencia distintos
    area1_.change.x = 0.13105;
    area1_.change.y = -0.415;
    area1_.change.z = 0.0;
    
    area2_.extraction.x = -0.13105;
    area2_.extraction.y = -0.36246;
    area2_.extraction.z = -0.1033;
    area2_.insertion.x = 0.13105;
    area2_.insertion.y = -0.36246;
    area2_.insertion.z = -0.1033;
    //se supone que change es el mismo punto pero visto en dos sistemas de referencia distintos
    area2_.change.x = 0.13105;
    area2_.change.y = -0.415;
    area2_.change.z = 0.0;

    //hacer un vector de areas
    areas.push_back(area1_);
    areas.push_back(area2_);
    
    num_areas_.data = 2;

    availableInsertion_.data.resize(2, 1); 
    availableExtraction_.data.resize(2, 1); 
    finished_.data.resize(2, 0);
    availableInsertion_pub_.publish(availableInsertion_);
    availableExtraction_pub_.publish(availableExtraction_);
    finished_pub_.publish(finished_); 

    num_areas_pub_.publish(num_areas_);
    area_pub_.publish(areas[n]);
    goal_pubs_[0].publish(areas[n].change);
    areas[n].change.x = -areas[n].change.x;
    goal_pubs_[1].publish(areas[n].change);
    areas[n].change.x = -areas[n].change.x;
    msg_g1.data = true;
    msg_g2.data = false;
  }
  void phase_1(){
    goal_pubs_[0].publish(areas[n].insertion);
  }
  void phase_2(){
    goal_pubs_[1].publish(areas[n].extraction);
    //puntion
    availableInsertion_.data[n] = 0;
    availableExtraction_.data[n] = 0;
    //availableInsertion_pub_.publish(availableInsertion_);
    //availableExtraction_pub_.publish(availableExtraction_);
  }
  void phase_3(){
    std_msgs::Bool msg;
    msg_g2.data = true;
    //goal_gripper_pubs_[1].publish(msg);
  }
  void phase_4(){
    std_msgs::Bool msg;
    msg_g1.data = false;
    //goal_gripper_pubs_[0].publish(msg);
  }
  void phase_5(){
    areas[n].change.x = -areas[n].change.x;
    goal_pubs_[1].publish(areas[n].change);
    areas[n].change.x = -areas[n].change.x;
  }
  void phase_6(){
    goal_pubs_[0].publish(areas[n].change);
  }
  void phase_7(){
    std_msgs::Bool msg;
    msg_g1.data = true;
    //goal_gripper_pubs_[0].publish(msg);
  }
  void phase_8(){
    std_msgs::Bool msg;
    msg_g2.data = false;
    //goal_gripper_pubs_[1].publish(msg);

    finished_.data[n] = 1;
    finished_pub_.publish(finished_);
    n = n + 1;
    area_pub_.publish(areas[n]);
  }
  void vision_failure(){
    availableInsertion_.data[n] = 0;
    availableExtraction_.data[n] = 1;
  }
  void gripper_failure(){
    msg_g1.data = false;
    msg_g2.data = false;
  }
  void force_failure(){
    areas[n].insertion.z = areas[n].insertion.z - 0.02;
    goal_pubs_[0].publish(areas[n].insertion);
    areas[n].insertion.z = areas[n].insertion.z + 0.02;
  }
  void combination_failure(){
    areas[n].extraction.z = areas[n].extraction.z - 0.02;
    goal_pubs_[1].publish(areas[n].extraction);
    areas[n].extraction.z = areas[n].extraction.z + 0.02;
    availableInsertion_.data[n] = 1;
    availableExtraction_.data[n] = 0;
  }
  void publicador(){
    goal_gripper_pubs_[0].publish(msg_g1);
    goal_gripper_pubs_[1].publish(msg_g2);
    availableInsertion_pub_.publish(availableInsertion_);
    availableExtraction_pub_.publish(availableExtraction_);
    finished_pub_.publish(finished_); 
  }
private:
  ros::NodeHandle nh_;
  //sub
  //pub
  std::vector<ros::Publisher> goal_pubs_;
  std::vector<ros::Publisher> goal_gripper_pubs_;
  //pub in ros a std_msgs::UInt8MultiArray
  ros::Publisher finished_pub_;
  ros::Publisher availableInsertion_pub_;
  ros::Publisher availableExtraction_pub_;
  ros::Publisher num_areas_pub_;
  ros::Publisher area_pub_;
  //variables
  std::vector<uma_fp_control::stitchArea> areas;
  uma_fp_control::stitchArea area1_;
  uma_fp_control::stitchArea area2_;
  std_msgs::Int32 num_areas_;
  geometry_msgs::Point phantomPose_;
  std::vector<double> phantomPoseRef_;
  std_msgs::UInt8MultiArray availableInsertion_;
  std_msgs::UInt8MultiArray availableExtraction_;
  std_msgs::UInt8MultiArray finished_;
  std_msgs::Bool msg_g1;
  std_msgs::Bool msg_g2;
  int n = 0;
  Eigen::MatrixXd Tpaso_ = Eigen::Matrix4d::Zero();;
};

int main(int argc, char** argv) {
  // Inicializa el nodo ROS
  ROS_INFO("\r\n mock\r\n");
  ros::init(argc, argv, "ontology_mock");
  // Crea un manejador de nodo ROS
  ros::NodeHandle nh;
  ros::Rate rate(125); 
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
  //Para realizar la lectura por pantalla
  system("stty raw"); //evita presionar enter tras cada letra
  bool salir=false; //sale tras pulsar x
  ROS_INFO("\r\n Introduzca un 0 primero para inicializar el robot\r\n");
  while (ros::ok() && !salir)
  {
    ROS_INFO("\r\n Introduzca la fase en la que está: 1-8. v= vision failure, g= gripper failure, f= force failure, c= combination failure\r\n");
    char input=getchar();
    switch (input)
    {
        case '0': 
            coord.initE();
            break;
        case '1': 
            coord.phase_1();
            break;  
        case '2':
            coord.phase_2();
            break;  
        case '3':
            coord.phase_3();
            break;  
        case '4':
            coord.phase_4();
            break;  
        case '5':
            coord.phase_5();
            break;  
        case '6':
            coord.phase_6();
            break;  
        case '7':
            coord.phase_7();
            break;  
        case '8':
            coord.phase_8();
            break;
        case 'v':
            coord.vision_failure();
            break;
        case 'g':
            coord.gripper_failure();
            break;
        case 'f':
            coord.force_failure();
            break;
        case 'c':
            coord.combination_failure();
            break;
        case 'x':
            salir = true;
            system("stty cooked");
            break;         
    }
    coord.publicador();
    ROS_INFO("\r\n Enviando tecla. Pulsa x para salir y r para resetear \r\n");
  }
  ros::spin();
  return 0;
}