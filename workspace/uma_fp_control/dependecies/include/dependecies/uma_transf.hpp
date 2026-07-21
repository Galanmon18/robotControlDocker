#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen_conversions/eigen_msg.h>

#include "cmath"
#include "math.h"

/*#include "geometry_msgs/PoseStamped.h" 
#include "omni_msgs/OmniButtonEvent.h"*/
#pragma once
class UMA_trans
{
	public:
		UMA_trans();
		~UMA_trans();
		
		Eigen::MatrixXd desp(std::vector<double> D);
		Eigen::MatrixXd rotX(double rad);
		Eigen::MatrixXd rotY(double rad);
		Eigen::MatrixXd rotZ(double rad);
		Eigen::MatrixXd etrans(double alpha, double beta, double rho);
};