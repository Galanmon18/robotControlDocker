#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <string>
#include <eigen_conversions/eigen_msg.h>
//#include <Eigen/Core>
//#include <Eigen/Geometry>

#pragma once
class PoseDesJM
{
	public:
		PoseDesJM();//(std::vector<double> P_Fp, Eigen::MatrixXd T_TTP_origen, std::vector<double> DP, std::vector<double> RPY); //ros::NodeHandle n
		~PoseDesJM();
	
		//funciones
		//void computePoseDes(Eigen::MatrixXd T_dest, Eigen::MatrixXd R_Fp);
		Eigen::MatrixXd computePoseDes(Eigen::MatrixXd T_dest, Eigen::MatrixXd R_Fp);
		//PROBAR HACERLO VOID SINO HACER QUE DEVUELVA UNA ARRAY DE DOUBLE CON LOS 6 VALORES
		
		//Variables
		Eigen::MatrixXd T_TTP;
		Eigen::MatrixXd Fp_Tdest;

		std::vector<double> vX = {0.0,0.0,0.0};
		std::vector<double> vY = {0.0,0.0,0.0};
		std::vector<double> vZ = {0.0,0.0,0.0};

		double roll_dest, pitch_dest, yaw_dest;
};
