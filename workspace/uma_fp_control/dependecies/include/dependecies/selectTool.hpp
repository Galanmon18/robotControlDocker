#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <string>
#include <eigen_conversions/eigen_msg.h>
#include "dependecies/uma_transf.hpp"

class selectTool
{
	public:
		selectTool(); //ros::NodeHandle n
		~selectTool();
		
		//funciones
		void computeTwrist(int type, double p_estimado, double tool_length, UMA_trans* tr);
		
		//variables
        Eigen::MatrixXd E_T_TTP;
        Eigen::MatrixXd E_T_Fp;
        std::vector<double> TCP;
		std::vector<double> DFP;
};