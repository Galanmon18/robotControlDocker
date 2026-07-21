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
#include <algorithm>
#include <fstream>
#include <sstream>
#include <ros/package.h>

class coordinator {
public:
  coordinator(const std::vector<std::string>& robot_names){
    // Inicializar matrices
    T1_ = Eigen::Matrix4d::Identity();
    T2_ = Eigen::Matrix4d::Identity();
    // Cargar matriz de transformación
    loadTransformMatrix();
    // Subscripciones a los topics
    haptic_sub_ = nh_.subscribe("/bleending/sangre_centroides", 1000, &coordinator::cmd_haptic, this);
    pose_r1_sub_ = nh_.subscribe(robot_names[0] + "/pose_topic", 1000, &coordinator::cmd_pose1, this);
    pose_r2_sub_ = nh_.subscribe(robot_names[1] + "/pose_topic", 1000, &coordinator::cmd_pose2, this);
    //pub
    goal_pub_ = nh_.advertise<geometry_msgs::Point>("coordinator/goal_position", 1000);
    fulcrum_pub_ = nh_.advertise<geometry_msgs::Point>("coordinator/fulcrum_position", 1000);
  }
  void cmd_haptic(const geometry_msgs::Point& msg){
    //ROS_INFO_STREAM("-----entro cmd_haptic----");
    /*stitch_.x = T1_(0,3) + msg.x;
    stitch_.y = T1_(1,3) + msg.y;
    stitch_.z = T1_(2,3) + msg.z;*/
    //camera_Pdes_ <<-0.07,0.05,0.25, 1;
    camera_Pdes_ << msg.x,msg.y,msg.z,1;//0.0408,0.0495,0.154,1; //0.25,0.383,-1.63
    Eigen::Vector4d camera_Pdes_transformed = T_transform_ * T2_ * camera_Pdes_;
    std::cout << "camera_Pdes_transformed_1=[" << camera_Pdes_transformed << "]" << std::endl;
    Tc_ << 1, 0, 0, -0.0,//-0.0206,
            0, 1, 0, -0.025,//-0.0184,
            0, 0, 1,  0.002,//-0.0115,
            0, 0, 0, 1;
    camera_Pdes_transformed = Tc_ * camera_Pdes_transformed;
    std::cout << "************Pdes************" << std::endl;
    std::cout << "camera_Pdes_=[" << camera_Pdes_ << "]" << std::endl;
    std::cout << "camera_Pdes_transformed=[" << camera_Pdes_transformed << "]" << std::endl;
    stitch_.x = camera_Pdes_transformed(0);
    stitch_.y = camera_Pdes_transformed(1);
    stitch_.z = camera_Pdes_transformed(2);
    goal_pub_.publish(stitch_);
  }
  void cmd_pose1(const std_msgs::Float64MultiArray::ConstPtr& msg){
    Eigen::MatrixXd matrix(4, 4);
    // Copiar los datos del mensaje a la matriz Eigen
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
    // Copiar los datos del mensaje a la matriz Eigen
    int index = 0;
    for (int j = 0; j < 4; ++j) {
        for (int i = 0; i < 4; ++i) {
            matrix(i,j) = msg->data[index++];
        }
    }
    T2_ = matrix;
    //Get position of the tool for fulcrm stimation
    fulcrumVec_ << T2_(0,3), T2_(1,3), T2_(2,3), 1;
    //std::cout << "fulcrumVec_: " << fulcrumVec_ << std::endl;
    Eigen::Vector4d fulcrumVec_transformed = T_transform_ * fulcrumVec_;
    fulcrumPose_.x = fulcrumVec_transformed(0);
    fulcrumPose_.y = fulcrumVec_transformed(1);
    fulcrumPose_.z = fulcrumVec_transformed(2);
    //std::cout << "fulcrumPose_: " << fulcrumPose_ << std::endl;
    fulcrum_pub_.publish(fulcrumPose_);
    //TENGO QUE PASARLO AL SISTEMA DE REFERENCIA OTRO
  }
//----------------------------------------------------------
  void loadTransformMatrix() {
    // Obtener la ruta del CSV desde parámetros ROS o usar ruta por defecto
    std::string csv_path;
    if (!nh_.getParam("transform_matrix_csv_path", csv_path)) {
      // Ruta por defecto: en el directorio config del paquete
      csv_path = ros::package::getPath("uma_fp_control") + "/config/transform_matrix.csv";
      ROS_WARN_STREAM("No se especificó ruta CSV, usando: ");
    }
    
    T_transform_ = loadTransformFromCSV(csv_path);
    if (T_transform_.isZero()) {
      ROS_ERROR("Error cargando matriz CSV, usando matriz por defecto");
      T_transform_ = Eigen::Matrix4d::Identity();
    }
  }
  
  Eigen::Matrix4d loadTransformFromCSV(const std::string& filename) {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Zero();
    std::ifstream file(filename);
    
    if (!file.is_open()) {
      ROS_ERROR_STREAM("No se pudo abrir el archivo CSV: " << filename);
      return matrix; // Devuelve matriz de ceros para indicar error
    }
    
    std::string line;
    int row = 0;
    bool success = true;
    
    while (std::getline(file, line) && row < 4) {
      // Eliminar espacios en blanco
      line.erase(std::remove_if(line.begin(), line.end(), ::isspace), line.end());
      
      if (line.empty()) continue; // Saltar líneas vacías
      
      std::stringstream ss(line);
      std::string cell;
      int col = 0;
      
      while (std::getline(ss, cell, ',') && col < 4) {
        try {
          matrix(row, col) = std::stod(cell);
          col++;
        } catch (const std::exception& e) {
          ROS_ERROR_STREAM("Error convirtiendo valor en fila " << row << ", columna " << col << ": " << cell);
          success = false;
          break;
        }
      }
      
      if (col != 4) {
        ROS_ERROR_STREAM("Fila " << row << " no tiene 4 columnas (tiene " << col << ")");
        success = false;
        break;
      } 
      row++;
    }
    
    file.close();
    
    if (row != 4 || !success) {
      ROS_ERROR_STREAM("Error: El archivo CSV debe tener exactamente 4 filas y 4 columnas");
      return Eigen::Matrix4d::Zero();
    }
    
    ROS_INFO_STREAM("Matriz de transformacion cargada exitosamente desde: " << filename);
    ROS_INFO_STREAM("Matriz:\n" << matrix);
    return matrix;
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
  ros::Publisher goal_pub_;
  ros::Publisher fulcrum_pub_;
  //variables
  geometry_msgs::Point stitch_;
  geometry_msgs::Point fulcrumPose_;
  std::vector<double> phantomPoseRef_;
  Eigen::MatrixXd T1_;
  Eigen::MatrixXd T2_;
  Eigen::Matrix4d T_transform_, Tc_;
  Eigen::MatrixXd dest_;
  Eigen::Matrix4d Tdesp_ = Eigen::Matrix4d::Zero();
  Eigen::Vector4d fulcrumVec_, camera_Pdes_;
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