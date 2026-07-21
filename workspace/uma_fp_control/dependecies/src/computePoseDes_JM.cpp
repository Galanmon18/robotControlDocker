// #include <dependecies/computePoseDes_JM.hpp>
#include "dependecies/computePoseDes_JM.hpp"

// Eigen::MatrixXd InversaMatriz(Eigen::MatrixXd R);

PoseDesJM::PoseDesJM()//Constructor
{
	ROS_INFO_STREAM("---PoseDesJM---");
		//PROBAR HACERLO VOID SINO HACER QUE DEVUELVA UNA ARRAY DE DOUBLE CON LOS 6 VALORES
}	

PoseDesJM::~PoseDesJM()
{
    ROS_INFO_STREAM("Leaving gently PoseDesJM...");
}
//void PoseDesJM::computePoseDes(Eigen::MatrixXd T_dest, Eigen::MatrixXd R_Fp)
Eigen::MatrixXd  PoseDesJM::computePoseDes(Eigen::MatrixXd T_dest, Eigen::MatrixXd R_Fp)
{
    Eigen::MatrixXd W_Rdest(4,4);
		Eigen::MatrixXd R_Rdest(4,4);
    //R_Fp es Fp
    Fp_Tdest = R_Fp.inverse() * T_dest;

    vZ[0]=Fp_Tdest(0,3);
    vZ[1]=Fp_Tdest(1,3);
    vZ[2]=Fp_Tdest(2,3);
    auto M_vZ = sqrt(pow(vZ[0],2) + pow(vZ[1],2) + pow(vZ[2],2));
    vZ[0]= vZ[0] / M_vZ;
    vZ[1]= vZ[1] / M_vZ;
    vZ[2]= vZ[2] / M_vZ;

    vX[0]= vZ[2];
    vX[1]= 0;
    vX[2]= -vZ[0];
    auto M_vX = sqrt(pow(vX[0],2) + pow(vX[1],2) + pow(vX[2],2));
    vX[0]= vX[0] / M_vX;
    vX[1]= vX[1] / M_vX;
    vX[2]= vX[2] / M_vX;

    vY[0]= vZ[1]*vX[2]-vZ[2]*vX[1];
    vY[1]= vZ[2]*vX[0]-vZ[0]*vX[2];
    vY[2]= vZ[0]*vX[1]-vZ[1]*vX[0];
    auto M_vY = sqrt(pow(vY[0],2) + pow(vY[1],2) + pow(vY[2],2));
    vY[0]= vY[0] / M_vY;
    vY[1]= vY[1] / M_vY;
    vY[2]= vY[2] / M_vY;

    W_Rdest << vX[0],vY[0],vZ[0],0,vX[1],vY[1],vZ[1],0,vX[2],vY[2],vZ[2],0,0,0,0,1;

    R_Rdest = R_Fp * W_Rdest;
    //-----
    /*double roll = atan2(R_Rdest(2, 1), R_Rdest(2, 2));
    double pitch = asin(-R_Rdest(2, 0));
    double yaw = atan2(R_Rdest(1, 0), R_Rdest(0, 0));
    std::cout << "angles 1->" <<"roll=" <<roll << ", "<<"pitch=" <<pitch << ", " <<"yaw=" << yaw << std::endl;
    roll=std::atan2(T_dest(2,1),T_dest(2,2));
    pitch=std::atan2(-T_dest(2,0),sqrt(T_dest(0,0)*T_dest(0,0)+T_dest(1,0)*T_dest(1,0)));
    yaw=std::atan2(T_dest(1,0),T_dest(0,0));
    std::cout << "angles 2->" <<"roll=" <<roll << ", "<<"pitch=" <<pitch << ", " <<"yaw=" << yaw << std::endl;*/

    /*double roll = atan2(-Fp_Tdest(1,3), Fp_Tdest(2,3));
    double pitch = atan2(Fp_Tdest(0,3), Fp_Tdest(2,3));
    double yaw = 0;
    std::cout << "angles 7->" <<"roll=" <<roll << ", "<<"pitch=" <<pitch << ", " <<"yaw=" << yaw << std::endl;*/
    //angle1 = angles2+angle7
    /*Eigen::MatrixXd fulcrum_rot{Eigen::MatrixXd::Zero(3, 1)};
    Eigen::MatrixXd base_rot{Eigen::MatrixXd::Zero(3, 1)};
    fulcrum_rot << roll,pitch,yaw;
    base_rot = (R_Fp.topLeftCorner<3,3>()) * fulcrum_rot;*/
    /*W_Rdest =  rotZ(angles7(3)) * rotY(angles7(2)) * rotX(angles7(1));
    R_Rdest = R_Fp * W_Rdest;*/
    //-----
    R_Rdest(0,3) = T_dest(0,3);
    R_Rdest(1,3) = T_dest(1,3);
    R_Rdest(2,3) = T_dest(2,3);
    return R_Rdest;
}