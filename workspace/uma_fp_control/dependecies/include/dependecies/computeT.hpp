#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <string>
#include "geometry_msgs/Point.h"
#include <geometry_msgs/Twist.h> 
#include <eigen_conversions/eigen_msg.h>
#include "dependecies/uma_transf.hpp"
#include "dependecies/computePoseDes_JM.hpp"
#include "dependecies/readRobotTransform.hpp"
class Cinematic{
        private:
                const double PI = 3.141592653589793;
                const double TOLERANCE = 1e-2;
                const int MAX_ITER = 100;
                double alpha, alpha_current,beta, beta_current, rho;
                geometry_msgs::Point fulcrum_point_goal;
                geometry_msgs::Point fulcrum_point_current;
                geometry_msgs::Point fulcrum_point_now;
                geometry_msgs::Point endeffector_point_goal;
                Eigen::Vector4d eigen_fulcrum_point_current;
                Eigen::Vector4d eigen_fulcrum_point_goal;
                Eigen::Vector3d fulcrum_difference_position;
        public:
                Cinematic();
                ~Cinematic();
                
                //funciones
                void computeCinematicTf(Eigen::MatrixXd T_dest, Eigen::MatrixXd T_effector, Eigen::MatrixXd T_TipTopPosition, geometry_msgs::Point PositionStitches,
                geometry_msgs::Point last_PositionStitches, std::string topic_prefix,
                Eigen::MatrixXd E_T_M, Eigen::MatrixXd E_T_Fp, Eigen::MatrixXd T_FpIn,
                double p_estimado, bool flagFp,bool flagForce, UMA_trans* tr,PoseDesJM* Pdest);

                void computePivotPose(Eigen::MatrixXd T_dest, Eigen::MatrixXd T_effector, Eigen::MatrixXd T_TTP, Eigen::MatrixXd T_Fp, geometry_msgs::Point PositionStitches,double p_estimado,UMA_trans* tr,PoseDesJM* Pdest);

                void computeCinematicAlphaBeta(Eigen::MatrixXd T_dest, Eigen::MatrixXd T_effector, Eigen::MatrixXd T_TipTopPosition, geometry_msgs::Point PositionStitches,
                geometry_msgs::Point last_PositionStitches, Eigen::MatrixXd E_T_M, Eigen::MatrixXd E_T_Fp, Eigen::MatrixXd T_FpIn,
                double p_estimado,double tool_length, bool flagFp, UMA_trans* tr);

                void computeCinematicIncrement(Eigen::MatrixXd T_dest, Eigen::MatrixXd T_effector, Eigen::MatrixXd T_TipTopPosition, geometry_msgs::Point PositionStitches,
                geometry_msgs::Point last_PositionStitches, Eigen::MatrixXd E_T_M, Eigen::MatrixXd E_T_Fp, Eigen::MatrixXd T_FpIn,
                double p_estimado,double tool_length, bool flagFp, UMA_trans* tr);

                //variables
                Eigen::MatrixXd T_TTP;
                Eigen::MatrixXd T_Es;
                Eigen::MatrixXd T_Fp;
                Eigen::MatrixXd TTP_dest;
                Eigen::MatrixXd TTP_dest_orientado;
}; 