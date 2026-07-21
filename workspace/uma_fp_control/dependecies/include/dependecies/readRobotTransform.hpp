#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <string>
#include <tf/transform_listener.h>
#include "geometry_msgs/Pose.h"
//#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>
//#include <tf_conversions/tf_eigen.h>
//#include "dependecies/uma_transf.hpp"
//#include <eigen_conversions/eigen_msg.h>

class TF_read{
	public:
		TF_read();//(std::string sensor_id); //ros::NodeHandle n
		~TF_read();
        Eigen::MatrixXd readTransform(std::string base,std::string tool0);
		std::vector<std::string> frames;
	private:
		ros::NodeHandle n;
		ros::Subscriber subForceTorque;
		ros::Subscriber subForceTorque2;
		tf::TransformListener listener;
        tf::StampedTransform tf_pose;
        geometry_msgs::Pose pose_msg;
		tf::Quaternion quaternion;
        double roll, pitch, yaw;
        double X, Y, Z;
		double qX, qY, qZ, qW;
        std::vector<double> D;
};