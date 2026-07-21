#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <string>
#include "dependecies/ur_script.h"
#include "dependecies/uma_transf.hpp"
#include <eigen_conversions/eigen_msg.h>
#include "geometry_msgs/Pose.h"

class Init
{
	public:
		Init();
		~Init();
		
		//funciones
		void initialize(std::vector<double> punto, std::vector<double> tcp, ur_script * ur);
		void initializePolar(std::vector<double> polar_angles, Eigen::MatrixXd T_Fp, std::vector<double> tcp, ur_script * ur, UMA_trans* tr);

		//Const
		const double PI = 3.141592653589793;

		//variables
		geometry_msgs::Pose pose;
};