#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <string>
#include <eigen_conversions/eigen_msg.h>

class ErrorPose
{
	public:
		ErrorPose();
		~ErrorPose();
		
		//funciones
		//std::vector<double> computeError(std::vector<double> Dest, std::vector<double> Actual);
		std::vector<double> computeError(Eigen::MatrixXd Dest, Eigen::MatrixXd Actual);
		std::vector<double> computeErrorTf(const Eigen::MatrixXd& Dest, const Eigen::MatrixXd& Actual);;
		
		std::vector<double> error={0,0,0,0,0,0};
	private:
		double RPY[3];
		double RPYdest[3];
};