//
#include <dependecies/fulcrumEstimation.hpp>
fulcrum::fulcrum(){
	//ROS_INFO_STREAM("---fulcrum---");
}
fulcrum::~fulcrum(){
    //ROS_INFO_STREAM("Leaving gently fp...");
}
double fulcrum::computeFulcrum(const Eigen::Vector3d forceRobotEffector, const Eigen::Vector3d torqueRobotEffector, double tool, double p_anterior){
    double forceMagnitude = forceRobotEffector.norm();
    double torqueMagnitude = torqueRobotEffector.norm();
    double p_estimado = 0.0;
    if (forceMagnitude != 0) {
        p_estimado = torqueMagnitude / forceMagnitude;
        if (p_estimado>tool)
        {
            p_estimado = p_anterior;
        }
    }else{
        p_estimado = p_anterior;
    }
    return p_estimado;
}