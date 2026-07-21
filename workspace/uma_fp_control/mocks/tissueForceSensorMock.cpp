// lee por teclado comandos de teleoperación
// el resultado se manda por el topic cmd_key
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>
#include <sstream>
#include <stdlib.h>
#include <string.h>

// Variables
Eigen::MatrixXd T_ = Eigen::MatrixXd::Zero(4, 4); // Inicializamos T_ en cero
void cmd_pose(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    Eigen::MatrixXd matrix(4, 4);
    int index = 0;
    for (int j = 0; j < 4; ++j) {
        for (int i = 0; i < 4; ++i) {
            matrix(i, j) = msg->data[index++];
        }
    }
    T_ = matrix; // Asignamos la matriz recibida a T_
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "tissue_force_Moch");
    ros::NodeHandle n;
    ros::Subscriber pose_sub_ = n.subscribe("pose_topic", 1000, cmd_pose); // Corregido aquí
    ros::Publisher pub_ = n.advertise<geometry_msgs::Point>("tissue_force_topic", 1000);
    while (ros::ok()) {
        // Solo calculamos la intersección si T_ ha sido inicializada correctamente
        if (T_.size() != 0) {
            geometry_msgs::Point HEXForce;
            Eigen::Vector3d pF = T_.block<3, 1>(0, 3);
            HEXForce.x = 0;
            HEXForce.y = 0;
            HEXForce.z = (pF[2] > 0.5) ? 0.5 * (pF[2] - 0.5) : (pF[2] < 0.0) ? 0.5 * (pF[2] - 0.0) : 0;
            pub_.publish(HEXForce);
        }
        ros::spinOnce();
    }
    return 0;
}
