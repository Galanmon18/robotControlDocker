// lee por teclado comandos de teleoperación
// el resultado se manda por el topic cmd_key
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"
#include <geometry_msgs/Point.h> 
#include <geometry_msgs/Twist.h> 
#include <Eigen/Dense>
#include <sstream>
#include <stdlib.h>
#include <string.h>

// Variables
Eigen::MatrixXd T_ = Eigen::MatrixXd::Zero(4, 4); // Inicializamos T_ en cero
bool wait = true;
void cmd_pose(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    Eigen::MatrixXd matrix(4, 4);
    int index = 0;
    for (int j = 0; j < 4; ++j) {
        for (int i = 0; i < 4; ++i) {
            matrix(i, j) = msg->data[index++];
        }
    }
    T_ = matrix;
    wait = false;
}
Eigen::MatrixXd desp(std::vector<double> D) {
    Eigen::MatrixXd T(4, 4);
    T << 1, 0, 0, D[0],
         0, 1, 0, D[1],
         0, 0, 1, D[2],
         0, 0, 0, 1;
    return T;
}
Eigen::Vector3d intersection(const Eigen::Matrix4d& T1, const Eigen::Matrix4d& T2, double rho) {
    Eigen::Vector3d P1 = T1.block<3, 1>(0, 3); //TE
    Eigen::Vector3d P2 = T2.block<3, 1>(0, 3);
    Eigen::Vector3d ray = P2 - P1; // Vector de dirección del rayo
    Eigen::Vector3d normal(0, 0, 1); // Normal del plano (perpendicular al eje Z)
    // Calculamos el denominador del parámetro t
    double denominator = normal.dot(ray); // Producto punto entre normal y ray
    // Verificamos si la línea es paralela al plano
    if (denominator == 0) {
        return Eigen::Vector3d(NAN, NAN, NAN); // No hay intersección, retorno de NaN
    }
    // Calculamos t usando rho como la altura en Z
    double t = (rho - P1.z()) / ray.z(); // rho - altura del plano; ray.z() es la diferencia en Z
    // Verificamos si t es válido para asegurar que la intersección está en el segmento
    if (t < 0 || t > 1) {
        return Eigen::Vector3d(NAN, NAN, NAN); // Intersección fuera del segmento definido por P1 y P2
    }
    // Calculamos el punto de intersección
    Eigen::Vector3d P_intersection = P1 + t * ray; // Punto de intersección
    return P_intersection; // Retornamos el punto de intersección
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "fulcrum_force_Moch");
    ros::NodeHandle n;
    double p_estimado_init;
    double L = 0.44305;
    std::string prefix;
    n.param<std::string>("prefix", prefix, "darel");
    n.param<double>("p_estimado", p_estimado_init, 0.2);
    ros::Subscriber pose_sub_ = n.subscribe("/"+prefix+"/pose_topic", 1000, cmd_pose); // Corregido aquí
    ros::Publisher pub_ = n.advertise<geometry_msgs::Twist>("/fulcrum_force_topic", 1000);
    geometry_msgs::Point fulcrum_point;
    fulcrum_point.x = -0.29795;
    fulcrum_point.y = -0.13287;
    fulcrum_point.z = 0;
    double margin = 0.005;
    while (ros::ok()) {
        // Solo calculamos la intersección si T_ ha sido inicializada correctamente
        if (T_.size() != 0) {
            Eigen::Matrix4d matrixTE = T_ * desp({0, 0, -L});
            Eigen::Vector3d pF = intersection(T_, matrixTE, p_estimado_init);
            geometry_msgs::Twist sensor;
            //force
            sensor.linear.x = (std::abs(pF[0] - fulcrum_point.x) > margin) ? 20 * (-pF[0] + fulcrum_point.x) : 0;
            sensor.linear.y = (std::abs(pF[1] - fulcrum_point.y) > margin) ? 20 * (-pF[1] + fulcrum_point.y) : 0;
            sensor.linear.z = 0;

            //torque
            Eigen::Vector3d forceRobotBase(sensor.linear.x, sensor.linear.y, sensor.linear.z);
            Eigen::Matrix3d R = matrixTE.topLeftCorner<3, 3>();
            Eigen::Vector3d v(0, 0, p_estimado_init);
            Eigen::Vector3d forceRobotE = R.inverse() * forceRobotBase;
            Eigen::Vector3d torqueRobotE = v.cross(forceRobotE);
            Eigen::Vector3d torqueRobotBase = R * torqueRobotE;
            sensor.angular.x = torqueRobotBase(0);
            sensor.angular.y = torqueRobotBase(1);
            sensor.angular.z = torqueRobotBase(2);
            //pub in "/fulcrum_force_topic"
            pub_.publish(sensor);
        }
        ros::spinOnce(); // Procesar los mensajes
    }
    return 0;
}
