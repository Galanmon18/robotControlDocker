#include <dependecies/initialize.hpp>

Init::Init()//Constructor
{
	ROS_INFO_STREAM("---Init---");
}

Init::~Init()
{
    ROS_INFO_STREAM("Leaving gently Init...");
}

void Init::initialize(std::vector<double> punto, std::vector<double> tcp, ur_script * ur)
{
    //ur_script ur;
    //Se comienza la ejecución con el TCP activo en TOOL0
    ur->set_tcp(tcp);
    ros::Duration(1.5).sleep();
    //ur.set_tcp(tcp);
    ROS_INFO_STREAM("Guardando la posicion del TCP");
    ros::Duration(0.5).sleep();
    // Movimiento del robot hacia un punto cómodo.
    ROS_INFO_STREAM("moviendo al punto");
    ur->movej(punto, true);
    ros::Duration(0.5).sleep();
    ur->stopl(1); 
}

/*void Init::initializePolar(std::vector<double> polar_angles, Eigen::MatrixXd T_Fp,std::vector<double> tcp, ur_script * ur, UMA_trans* tr)
{
    //ur_script ur;
    //Se comienza la ejecución con el TCP activo en TOOL0
    ur->set_tcp(tcp);
    ros::Duration(1.0).sleep();
    ROS_INFO_STREAM("Guardando la posicion del TCP");
    ros::Duration(0.5).sleep();
    // Movimiento del robot hacia un punto cómodo.
    T_Fp = tr->desp({T_Fp(0,3), T_Fp(1,3), T_Fp(2,3)});
    Eigen::MatrixXd T_init = T_Fp * tr->rotZ(polar_angles[0]) * tr->rotY(-PI/2-polar_angles[1]) * tr->desp({0,0, tcp[2]-polar_angles[2]});
    std::cout << "***********" << std::endl;
    std::cout << "T_Fp"<< T_Fp<< std::endl;
    std::cout << "T_init->"<< T_init << std::endl;
    //angles Z Y X
    pose.orientation.x=std::atan2(T_init(2,1),T_init(2,2));
    pose.orientation.y=std::atan2(-T_init(2,0),sqrt(T_init(0,0)*T_init(0,0)+T_init(1,0)*T_init(1,0)));
    pose.orientation.z=std::atan2(T_init(1,0),T_init(0,0));
    if (pose.orientation.x>(-0.001) && pose.orientation.x<(0.001)){
        pose.orientation.x = 0;
    }
    if (pose.orientation.y>(-0.001) && pose.orientation.y<(0.001)){
        pose.orientation.y = 0;
    }
    if (pose.orientation.z>(-0.001) && pose.orientation.z<(0.001)){
        pose.orientation.z = 0;
    }
    //position
    pose.position.x = T_init(0,3);
    pose.position.y = T_init(1,3);
    pose.position.z = T_init(2,3);
    std::cout << "T_init pose->"<< pose << std::endl;
    std::cout << "***********" << std::endl;
    ROS_INFO_STREAM("moviendo a punto");
    //ur->movel(pose, true);
    ros::Duration(9.5).sleep();
    ROS_INFO_STREAM("fin");
    ur->stopl(1); 
}*/