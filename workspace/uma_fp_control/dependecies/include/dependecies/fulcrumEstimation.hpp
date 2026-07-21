#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <string>
#include <eigen_conversions/eigen_msg.h>

class fulcrum
{
	public:
		fulcrum(); //ros::NodeHandle n
		~fulcrum();
		//funciones
		double computeFulcrum(const Eigen::Vector3d forceRobotEffector, const Eigen::Vector3d torqueRobotEffector,
								 double tool, double p_anterior);
		double p_estimado = 0;
};
