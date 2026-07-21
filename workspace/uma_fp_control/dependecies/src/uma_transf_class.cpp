#include "dependecies/uma_transf.hpp"

UMA_trans::UMA_trans() //Constructor
{
	//ROS_INFO_STREAM("---UMA_trans---");
}

UMA_trans::~UMA_trans()
{
    //ROS_INFO_STREAM("Leaving gentlyTRA...");
}

// ----------------------------------------------------------------------------------------------------- //
// --------------------------------------------- FUNCIONES ---------------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //

Eigen::MatrixXd UMA_trans::desp(std::vector<double> D)
{
    Eigen::MatrixXd T(4,4);
    T << 1,0,0,D[0],0,1,0,D[1],0,0,1,D[2],0,0,0,1;
    return T;
}

Eigen::MatrixXd UMA_trans::rotX(double rad)
{
	double s = std::sin(rad);
	double c = std::cos(rad);

	if (fabs(s)<0.0000000001)
	{
		s=0;
	}
	if (fabs(c)<0.0000000001)
	{
		c=0;
	}
    Eigen::MatrixXd T(4,4);
    T << 1,0,0,0,0,c,-s,0,0,s,c,0,0,0,0,1;
    return T;
}

Eigen::MatrixXd UMA_trans::rotY(double rad)
{
    double s = std::sin(rad);
	double c = std::cos(rad);

	if (fabs(s)<0.0000000001)
	{
		s=0;
	}
	if (fabs(c)<0.0000000001)
	{
		c=0;
	}
    Eigen::MatrixXd T(4,4);
    T << c,0,s,0,0,1,0,0,-s,0,c,0,0,0,0,1;
    return T;
}

Eigen::MatrixXd UMA_trans::rotZ(double rad)
{
    double s = std::sin(rad);
	double c = std::cos(rad);

	if (fabs(s)<0.0000000001)
	{
		s=0;
	}
	if (fabs(c)<0.0000000001)
	{
		c=0;
	}
    Eigen::MatrixXd T(4,4);
    T << c,-s,0,0,s,c,0,0,0,0,1,0,0,0,0,1;
    return T;
}

Eigen::MatrixXd UMA_trans::etrans(double alpha, double beta, double rho){
	double s_a = std::sin(alpha);
	double c_a = std::cos(alpha);
	double s_b = std::sin(beta);
	double c_b = std::cos(beta);
	//vector double
	Eigen::Vector3d s(s_a,s_b,0);
	Eigen::Vector3d c(c_a,c_b,0);
	Eigen::MatrixXd T(4,4);
    T << -c(1)*c(2), -s(1), -c(1)*s(2), rho*c(1)*s(2),
			-c(2)*s(1), c(1), -s(1)*s(2), rho*s(1)*s(2),
			s(2), 0, -c(2), rho*c(2),
			0,0,0,1;
    return T;
}
