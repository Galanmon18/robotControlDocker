#include "dependecies/readRobotTransform.hpp"


TF_read::TF_read(){
	//ROS_INFO_STREAM("---TF_read---");
}

TF_read::~TF_read(){
    //ROS_INFO_STREAM("Leaving gently TF_read...");
}

Eigen::MatrixXd TF_read::readTransform(std::string base,std::string tool0){
    Eigen::MatrixXd T(4,4);
    try{
        listener.lookupTransform(base, tool0, ros::Time(0), tf_pose);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    //-------
    // Obtiene los marcos de referencia conocidos
    /*listener.getFrameStrings(frames);
    // Imprime los marcos de referencia
    for (const auto& frame : frames) {
        ROS_INFO("Marco de referencia: %s", frame.c_str());
    }/**/
    //-------
    qX = tf_pose.getRotation().x();
    qY = tf_pose.getRotation().y();
    qZ = tf_pose.getRotation().z();
    qW = tf_pose.getRotation().w();
    X = tf_pose.getOrigin().x();
    Y = tf_pose.getOrigin().y();
    Z = tf_pose.getOrigin().z();
    T << (1-2*(qY*qY+qZ*qZ)), (2*(qX*qY-qW*qZ)), 2*(qX*qZ+qW*qY), X,
         (2*(qX*qY+qW*qZ)), (1-2*(qX*qX+qZ*qZ)), 2*(qY*qZ-qW*qX), Y,
         2*(qX*qZ-qW*qY), 2*(qY*qZ+qW*qX), (1-2*(qX*qX+qY*qY)), Z,
         0,0,0,1;

    return T;
}

/*int main(int argc, char **argv)
{
  ros::init(argc, argv, "TF_read");
  TF_read tfread;
  ros::Duration(10.5).sleep();
  Eigen::MatrixXd R(4,4);
  R = tfread.readTransform("alice_base", "alice_tool0");
  std::cout << "R" << std::endl;
  std::cout << R << std::endl;
  ros::spin();
  return 0;
}*/