#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string.h>
#include "std_msgs/UInt8MultiArray.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include <cmath>
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <iostream>
#include <vector>
#include <dependecies/ur_script.h>
#include <fstream>  // Incluye fstream para manejar archivos

// Variables globales
geometry_msgs::Point angles,angles2, pos,pos2, stitch_, inicio_;
geometry_msgs::Twist vel;
Eigen::Matrix4d T,T2;
std::vector<double> HEXForce = {0,0,0};
std::vector<double> HEXtorque = {0,0,0};
double AngleX[5000], AngleY[5000], AngleZ[5000], velX[5000], velY[5000], velZ[5000];
double poseX[5000], poseY[5000], poseZ[5000], velWX[5000], velWY[5000], velWZ[5000];
double forceX[5000], forceY[5000], forceZ[5000], torqueX[5000], torqueY[5000], torqueZ[5000];
double AngleXdes[5000], AngleYdes[5000], AngleZdes[5000];
double poseXdes[5000], poseYdes[5000], poseZdes[5000];
double sec[5000], u[5000];
double distInit[5000], distEnd[5000];
Eigen::MatrixXd TS(5000, 16), error_r(5000,6), TS2(5000, 16);
std::vector<double> velVector = {0,0,0,0,0,0};
int p = 0;
bool wait = true;
//callback ros
void cmd_error(const std_msgs::Float64MultiArray::ConstPtr& msg){
    //save de msg in a vector of 6 element
    for (int k = 0; k < 6; ++k){
        error_r(p,k) = msg->data[k];
    }
}
void cmd_pose1(const std_msgs::Float64MultiArray::ConstPtr& msg){
    // Crear una matriz Eigen::MatrixXd para almacenar los datos del mensaje PÙNTA HERRAMIENTA H
    Eigen::MatrixXd matrix(4, 4);
    // Copiar los datos del mensaje a la matriz Eigen
    int index = 0;
    for (int j = 0; j < 4; ++j) {
        for (int i = 0; i < 4; ++i) {
            matrix(i,j) = msg->data[index++];
        }
    }
    T = matrix;
    angles.x = (std::atan2(T(2,1),T(2,2)) );
    angles.y = std::atan2(-T(2,0),sqrt((T(0,0)*T(0,0))+(T(1,0)*T(1,0))));
    angles.z = std::atan2(T(1,0),T(0,0));
    pos.x = T(0,3);
    pos.y = T(1,3);
    pos.z = T(2,3);
    wait = false;
}
void cmd_vel(const std_msgs::Float64MultiArray::ConstPtr& msg){
    vel.linear.x = msg->data[0];
    vel.linear.y = msg->data[1];
    vel.linear.z = msg->data[2];
    vel.angular.x = msg->data[3];
    vel.angular.y = msg->data[4];
    vel.angular.z = msg->data[5];
}
void cmd_pose2(const std_msgs::Float64MultiArray::ConstPtr& msg){
    // Crear una matriz Eigen::MatrixXd para almacenar los datos del mensaje EFECTO FINAL
    Eigen::MatrixXd matrix(4, 4);
    // Copiar los datos del mensaje a la matriz Eigen
    int index = 0;
    for (int j = 0; j < 4; ++j) {
        for (int i = 0; i < 4; ++i) {
            matrix(i,j) = msg->data[index++];
        }
    }
    angles2.x = (std::atan2(matrix(2,1),matrix(2,2)) );
    angles2.y = std::atan2(-matrix(2,0),sqrt((matrix(0,0)*matrix(0,0))+(matrix(1,0)*matrix(1,0))));
    angles2.z = std::atan2(matrix(1,0),matrix(0,0));
    pos2.x = matrix(0,3);
    pos2.y = matrix(1,3);
    pos2.z = matrix(2,3);
    T2 = matrix;
}
void cb_readForceTorque(const geometry_msgs::Twist::ConstPtr& msg){
    HEXForce[0]=(msg->linear.x);
    HEXForce[1]=(msg->linear.y);
    HEXForce[2]=(msg->linear.z);
    HEXtorque[0]=(msg->angular.x);
    HEXtorque[1]=(msg->angular.y);
    HEXtorque[2]=(msg->angular.z);
}
// Función para calcular la distancia entre dos puntos en 3D
double calcularDistancia(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) +
                     std::pow(p1.y - p2.y, 2) +
                     std::pow(p1.z - p2.z, 2));
}
// Función que devuelve las distancias al punto de inicio y al punto final
void distanciasPuntos(const geometry_msgs::Point& inicio, const geometry_msgs::Point& fin, const geometry_msgs::Point& actual, double& distInicio, double& distFinal) {
    distInicio = calcularDistancia(actual, inicio);
    distFinal = calcularDistancia(actual, fin);
}
// Función para establecer los valores deseados
void setDesiredValues(int p) {
    AngleXdes[p] = AngleXdes[0];
    AngleYdes[p] = AngleYdes[0];
    AngleZdes[p] = AngleZdes[0];
    poseXdes[p] = stitch_.x;
    poseYdes[p] = stitch_.y;
    poseZdes[p] = stitch_.z;
}

void cmd_haptic(const geometry_msgs::Point& msg){
    ROS_INFO_STREAM("-----entro cmd_haptic----");
    stitch_.x = T(0,3) + msg.x;
    stitch_.y = T(1,3) + msg.y;
    stitch_.z = T(2,3) + msg.z;
    //goal_pub_.publish(stitch_);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "identificarSistema");
    ros::NodeHandle nh;
    // Publicador y suscriptor
    ros::Publisher pub = nh.advertise<std_msgs::UInt8MultiArray>("identificarSistema", 1000);
    ros::Subscriber pose_r1_sub_ = nh.subscribe("alice/pose_topic", 1000, cmd_pose1);
    ros::Subscriber vel_sub_ = nh.subscribe("alice/velocity_topic", 1000, cmd_vel);
    ros::Subscriber pose_r2_sub_ = nh.subscribe("alice/effectorFinal_topic", 1000, cmd_pose2);
    ros::Subscriber error_r_ = nh.subscribe("alice/error_topic", 1000, cmd_error);
    ros::Subscriber force_ = nh.subscribe("/ur3e/rtde/force", 1000, cb_readForceTorque);
    ros::Subscriber haptic_sub_ = nh.subscribe("/haptic_topic", 1000, cmd_haptic);
    ros::Publisher goal_pub_ = nh.advertise<geometry_msgs::Point>("/alice/coordinator/goal_position", 1000);
    ros::Rate loop_rate(125);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    // Inicializar valores deseados
    /*stitch_.x = 0.16; //0.10763
    stitch_.y = -0.38; //-0.29975
    stitch_.z = -0.01; //0.31267 */
    AngleXdes[p] = (0);
    AngleYdes[p] = (0);
    AngleZdes[p] = (0);
    // Inicializar valores actuales
    AngleX[p] = (0);
    AngleY[p] = (0);
    AngleZ[p] = (0);
    poseX[p] = (0);
    poseY[p] = (0);
    poseZ[p] = (0);
    velWX[p] = (0);
    velWY[p] = (0);
    velWZ[p] = (0);
    velX[p] = (0);
    velY[p] = (0);  
    velZ[p] = (0);
    //initi pos y angle 
    pos.x = 0;
    pos.y = 0;
    pos.z = 0;
    angles.x = 0;
    angles.y = 0;
    angles.z = 0;
    //calcular longitudes
    double distInicio, distFinal;
    while (wait){
        //wait
    }
    inicio_.x = pos.x;
    inicio_.y = pos.y;
    inicio_.z = pos.z;
    stitch_.x = inicio_.x - 0.035;
    stitch_.y = inicio_.y + 0.0;
    stitch_.z = inicio_.z - 0.0;
    poseXdes[p] = stitch_.x;
    poseYdes[p] = stitch_.y;
    poseZdes[p] = stitch_.z;
    auto primeTime = std::chrono::high_resolution_clock::now();
    std::cout << "vamos al while" << std::endl;
    while (ros::ok())
    {
        goal_pub_.publish(stitch_);
        setDesiredValues(p);
        //valores
        velWX[p] = vel.angular.x;
        velWY[p] = vel.angular.y;
        velWZ[p] = vel.angular.z;
        velX[p] = vel.linear.x;
        velY[p] = vel.linear.y; 
        velZ[p] = vel.linear.z;

        AngleX[p] = angles.x;
        AngleY[p] = angles.y;
        AngleZ[p] = angles.z;
        poseX[p] = pos.x;
        poseY[p] = pos.y;
        poseZ[p] = pos.z;
        //longitud
        distanciasPuntos(inicio_, stitch_, pos, distInicio, distFinal);
        distInit[p] = distInicio;
        distEnd[p] = distFinal;
        // Calcular el tiempo transcurrido
        auto currentTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time = currentTime - primeTime;
        sec[p] = (time.count());
        // Guardar en TS la matriz de transformación T
        std::vector<double> vector(T.data(), T.data() + T.size());
        TS.row(p) = Eigen::Map<Eigen::Matrix<double, 1, 16>>(vector.data(), 1, vector.size());

        std::vector<double> vector2(T2.data(), T2.data() + T2.size());
        TS2.row(p) = Eigen::Map<Eigen::Matrix<double, 1, 16>>(vector2.data(), 1, vector2.size());
        // Guardar matriz effector final
        // Guardar fuerza y torque
        forceX[p] = HEXForce[0];
        forceY[p] = HEXForce[1];
        forceZ[p] = HEXForce[2];
        torqueX[p] = HEXtorque[0];
        torqueY[p] = HEXtorque[1];
        torqueZ[p] = HEXtorque[2];
        if (sec[p] > 5) {
            //------
            std::ofstream fich("Roll.csv");
            if (!fich){
                std::cout << "Error al abrir ejemplo.dat\n";
                exit(EXIT_FAILURE);
            }
                for (int k = 0; k < p; ++k){
                fich << AngleX[k] << std::endl;
            }
            std::ofstream fich2("Pitch.csv");
            if (!fich2){
                std::cout << "Error al abrir ejemplo.dat\n";
                exit(EXIT_FAILURE);
            }
                for (int k = 0; k < p; ++k){
                fich2 << AngleY[k] << std::endl;
            }
            std::ofstream fich3("Yaw.csv");
            if (!fich3){
                std::cout << "Error al abrir ejemplo.dat\n";
                exit(EXIT_FAILURE);
            }
                for (int k = 0; k < p; ++k){
                fich3 << AngleZ[k] << std::endl;
            }
            std::ofstream fich4("X.csv");
            if (!fich4){
                std::cout << "Error al abrir ejemplo.dat\n";
                exit(EXIT_FAILURE);
            }
                for (int k = 0; k < p; ++k){
                fich4 << poseX[k] << std::endl;
            }
            std::ofstream fich5("Y.csv");
            if (!fich5){
                std::cout << "Error al abrir ejemplo.dat\n";
                exit(EXIT_FAILURE);
            }
            for (int k = 0; k < p; ++k){
                fich5 << poseY[k] << std::endl;
            }
            std::ofstream fich6("Z.csv");
            if (!fich6){
                std::cout << "Error al abrir ejemplo.dat\n";
                exit(EXIT_FAILURE);
            }
            for (int k = 0; k < p; ++k){
                fich6 << poseZ[k] << std::endl;
            }
            std::ofstream fich7("T_ttp_actual.csv");
            if (!fich7){
                std::cout << "Error al abrir ejemplo.dat\n";
                exit(EXIT_FAILURE);
            }
            for (int k = 0; k < p; ++k){
                for(int idm = 0; idm < 16; ++idm){
                    fich7 << TS(k,idm) << ",";
                }
                fich7 << std::endl;
            }
            /*std::ofstream fich8("time.csv");
            if (!fich8){
                std::cout << "Error al abrir ejemplo.dat\n";
                exit(EXIT_FAILURE);
            }
                for (int k = 0; k < p; ++k){
                fich8 << sec[k] << std::endl;
            }*/
            //guardar en ficheros los valores deseados de XYZ rX rY rZ
            std::ofstream fich9("velX.csv");
            if (!fich9){
                std::cout << "Error al abrir ejemplo.dat\n";
                exit(EXIT_FAILURE);
            }
            for (int k = 0; k < p; ++k){
                fich9 << velX[k] << std::endl;
            }
            std::ofstream fich10("velY.csv");
            if (!fich10){
                std::cout << "Error al abrir ejemplo.dat\n";
                exit(EXIT_FAILURE);
            }
            for (int k = 0; k < p; ++k){
                fich10 << velY[k] << std::endl;
            }
            std::ofstream fich11("velZ.csv");
            if (!fich11){
                std::cout << "Error al abrir ejemplo.dat\n";
                exit(EXIT_FAILURE);
            }
            for (int k = 0; k < p; ++k){
                fich11 << velZ[k] << std::endl;
            }
            std::ofstream fich12("wX.csv");
            if (!fich12){
                std::cout << "Error al abrir ejemplo.dat\n";
                exit(EXIT_FAILURE);
            }
            for (int k = 0; k < p; ++k){
                fich12 << velWX[k] << std::endl;
            }
            std::ofstream fich13("wY.csv");
            if (!fich13){
                std::cout << "Error al abrir ejemplo.dat\n";
                exit(EXIT_FAILURE);
            }
            for (int k = 0; k < p; ++k){
                fich13 << velWY[k] << std::endl;
            }
            std::ofstream fich14("wZ.csv");
            if (!fich14){
                std::cout << "Error al abrir ejemplo.dat\n";
                exit(EXIT_FAILURE);
            }
            for (int k = 0; k < p; ++k){
                fich14 << velWZ[k] << std::endl;
            }
            /*std::ofstream fich15("error.csv");
            if (!fich15){
                std::cout << "Error al abrir ejemplo.dat\n";
                exit(EXIT_FAILURE);
            }
            for (int k = 0; k < p; ++k){
                for(int idm = 0; idm < 6; ++idm){
                    fich15 << error_r(k,idm) << ",";
                }
                fich15 << std::endl;
            }*/
            //dist
            std::ofstream fich16("distInit.csv");
            if (!fich16){
                std::cout << "Error al abrir ejemplo.dat\n";
                exit(EXIT_FAILURE);
            }
                for (int k = 0; k < p; ++k){
                fich16 << distInit[k] << std::endl;
            }
            std::ofstream fich17("distEnd.csv");
            if (!fich17){
                std::cout << "Error al abrir ejemplo.dat\n";
                exit(EXIT_FAILURE);
            }
            for (int k = 0; k < p; ++k){
                fich17 << distEnd[k] << std::endl;
            }
            std::ofstream fich18("T_E_actual.csv");
            if (!fich18){
                std::cout << "Error al abrir ejemplo.dat\n";
                exit(EXIT_FAILURE);
            }
            for (int k = 0; k < p; ++k){
                for(int idm = 0; idm < 16; ++idm){
                    fich18 << TS2(k,idm) << ",";
                }
                fich18 << std::endl;
            }
            /*std::ofstream fich19("forceX.csv");
            if (!fich19){
                std::cout << "Error al abrir ejemplo.dat\n";
                exit(EXIT_FAILURE);
            }
            for (int k = 0; k < p; ++k){
                fich19 << forceX[k] << std::endl;
            }
            std::ofstream fich20("forceY.csv");
            if (!fich20){
                std::cout << "Error al abrir ejemplo.dat\n";
                exit(EXIT_FAILURE);
            }
            for (int k = 0; k < p; ++k){
                fich20 << forceY[k] << std::endl;
            }
            std::ofstream fich21("forceZ.csv");
            if (!fich21){
                std::cout << "Error al abrir ejemplo.dat\n";
                exit(EXIT_FAILURE);
            }
            for (int k = 0; k < p; ++k){
                fich21 << forceZ[k] << std::endl;
            }
            std::ofstream fich22("torqueX.csv");
            if (!fich22){
                std::cout << "Error al abrir ejemplo.dat\n";
                exit(EXIT_FAILURE);
            }
            for (int k = 0; k < p; ++k){
                fich22 << torqueX[k] << std::endl;
            }
            std::ofstream fich23("torqueY.csv");
            if (!fich23){
                std::cout << "Error al abrir ejemplo.dat\n";
                exit(EXIT_FAILURE);
            }
            for (int k = 0; k < p; ++k){
                fich23 << torqueY[k] << std::endl;
            }
            std::ofstream fich24("torqueZ.csv");
            if (!fich24){
                std::cout << "Error al abrir ejemplo.dat\n";
                exit(EXIT_FAILURE);
            }
            for (int k = 0; k < p; ++k){
                fich24 << torqueZ[k] << std::endl;
            }*/
            //------
            std::cout << " fin= " << sec[p] << std::endl;
            ros::Duration(1.0).sleep();
            exit(0);
            break;
        }
        p++;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}