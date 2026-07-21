#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <string>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Dense>
using Eigen::Vector3d;
#include "cmath"
#include "math.h"
#include <chrono>
#include <std_msgs/Bool.h>
#include "geometry_msgs/Point.h"
//#include <omni_msgs/OmniFeedback.h>
//#include <omni_msgs/OmniState.h>
//#include <omni_msgs/OmniButtonEvent.h>
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "uma_fp_control/ftResponse.h"
#include "uma_fp_control/zeroSensorpetition.h"
#include <dependecies/computeError.hpp>
#include <dependecies/computePID.hpp>
#include <dependecies/computePoseDes_JM.hpp>
#include <dependecies/computeT.hpp>
#include <dependecies/forceControlStrategy.hpp>
#include <dependecies/initialize.hpp>
#include <dependecies/selectTool.hpp>
#include <dependecies/ur_script.h>
#include <dependecies/fulcrumEstimation.hpp>
#include <dependecies/jacobianss.hpp>
#include "dependecies/alphaBegaGenerator.hpp"
#include <dependecies/fuzzySystem.hpp>
#include <tf/transform_listener.h>
#include "geometry_msgs/Pose.h"
#include "dependecies/hex_ft_udp.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <array>
#include <sstream>
#include <iomanip>

#define DEG_TO_RAD 0.017453293
#define RAD_TO_DEG 57.295779513

class jacobianControl
{       
	private:
                const double PI = 3.141592653589793;
                ur_script* ur;
                //La lista de inicialización en el constructor de una clase en C++ se utiliza para inicializar los miembros de la clase.
                 //No se utiliza para declarar variables, ya que las variables miembro deben haber sido declaradas previamente en la 
                 //definición de la clase. Para poder usarlo en la lista de inicialización, debe haber sido declarado en la clase
                UMA_trans* tr;
                Init* init;
                selectTool* tool;
                std::string prefix_in;
                PIDController* controlPosition;
                fulcrum* fulcrumEstimation;
                FTSensor* ftSensor;
                tf::TransformListener* listener; // Cambiar a puntero
                //urJacobian* jacob;
                //FuzzySystem* fuzzySystem; 
        public:
		jacobianControl(ur_script * urScript, UMA_trans *umaTf, Init *initRobot, PIDController *composePID, selectTool *selectTool, fulcrum *fp, FTSensor* ftSensor, tf::TransformListener* tf_listener, std::string t_prefix);
		~jacobianControl();
                //variables
                        //init
                std::vector<double> tool0 = {0., 0., 0., 0., 0., 0.};
                        //position control
                geometry_msgs::Point desPose;
                geometry_msgs::Point desPoseCom;
                bool desPoseReceived = false;
                bool newFulcrum = true;
                bool fulcrumReceived = false;
                bool show = false;
                bool inOrientation = false;
                geometry_msgs::Point fulcrum_position;
                geometry_msgs::Point fulcrum_point_des;
                geometry_msgs::Point fulcrum_point_current;
                double p_estimado;
                        //polar
                std::vector<double> des_polar_angles = {0,0,0};
                std::vector<double> current_polar_angles = {0,0,0};
                Eigen::Vector3d delta_polar, delta_polar_0, delta_position, delta_orientation;
                Eigen::VectorXd delta_cartesian, d;
                        //matrix
                Eigen::MatrixXd E_T_TTP;
                Eigen::MatrixXd E_T_Fp;
                Eigen::MatrixXd T_E;
                Eigen::MatrixXd T_Fp;
                Eigen::MatrixXd T_TTP;
                Eigen::MatrixXd Jacobian, inverseJacobian, TA;
                //force control
                //omni_msgs::OmniFeedback feedback_msg;
                //TCP
                int conn_fd_;
                int createServerSocket(int port);
                void sendPose(int conn_fd, const std::array<double, 6>& pose);
                //last
                std::vector<double> TCP;
                //ROS
                ros::NodeHandle nh_;
                ros::Subscriber goal_pos_sub_;
                ros::Subscriber initial_pose_sub_;
                ros::Subscriber force_sensor_sub_;
                ros::Subscriber fulcrum_sub_;
                ros::Subscriber abdomen_effector_force_sub_;
                ros::Subscriber tissue_force_sub_;
                ros::Publisher reset_sensor_pub_;
                ros::Publisher ttp_pub_;
                ros::Publisher vel_pub_;
                ros::Publisher te_pub_;
                ros::Publisher currentPolar_pub_;
                ros::Publisher desPolar_pub_;
                ros::Publisher deltaPolar_pub_;
                ros::Publisher fulcrum_pub_;
                ros::Publisher force_pub_;
                ros::Publisher base_force_pub_;
                ros::Publisher maxwell_pub_;
                ros::Publisher abdomen_force_pub_;
                ros::Publisher abdomen_effector_force_pub_;
                std_msgs::Float64MultiArray pose_, poseE_, array_vel, currentPolar_, desPolar_, deltaPolar_, fulcrum_position_, force_msg_, base_force_msg_, abdomen_force_effector_msg_, abdomen_force_msg_, maxwell_msg_;
                //time
                ros::Time start_time;
                ros::Time now;
                //force
                std::vector<double> forces, baseForces;
                std::vector<double> delta_force; //del control de fuerza
                double filter_alpha ;
                double error_F;
                double v_z;
                double Fz_filt;
                double error_integral;
                Eigen::Vector3d robotBaseForce, toolForce;
                Eigen::Vector3d forceRobotTTP;
                Eigen::Vector3d torqueRobotTTP;
                Eigen::Vector3d forceRobotBase;
                Eigen::Vector3d torqueRobotBase;
                std::vector<double> forceAbdomen, torqueAbdomen, forceTissue, torqueTissue;
                Eigen::Vector3d weight_base;
                Eigen::Vector3d tool_CoM;
                bool first_force_reading = true;
                struct FTResult{
                        Eigen::Vector3d F_comp_tool;
                        Eigen::Vector3d T_comp_tool;
                };
                Eigen::Vector3d bias_base;
                bool bias_initialized;
                // Posicion piel
                double z_tissue = -0.064;
                //posicion minima piel
                double z_min = -0.06944;//-0.071;
                double z_tissue_auto = -0.0645;
                //TF
                //tf::TransformListener* listener;
                tf::StampedTransform tf_pose;
                double X, Y, Z;
		double qX, qY, qZ, qW;
                std::string base_name;
                std::string tool_name;
                std::string efector_name;
                Eigen::MatrixXd readTransform(std::string base,std::string tool0);
                Eigen::MatrixXd computeJacobian(double a, double b, double p, double L);
                Eigen::MatrixXd computeInverseJacobian(double a, double b, double p, double L);
                Eigen::MatrixXd computeTconversion(double a, double b, double p, double L);
                Eigen::VectorXd velocityPropagation(const Eigen::VectorXd& delta_cartesian, const geometry_msgs::Point& fulcrum_point_current, const Eigen::MatrixXd& conversion_matrix);
                void roundDeltaCartesian(Eigen::VectorXd& delta_cartesian);
                //callback
                void cb_stitchCallback(const geometry_msgs::Point::ConstPtr& msg);
                void cb_readForceTorque(const geometry_msgs::Twist::ConstPtr& msg);
                void cb_fulcrumCallback(const geometry_msgs::Point::ConstPtr& msg);
                void cb_abdomenForceCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
                void cb_tissueForceCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
                //function
                void initializeRobot(int type, double p_estimado, double tool_length, std::vector<double> initPosition);
                void moveToDesireJacobOrientation_init(Eigen::MatrixXd T,  std::vector<double> polarCoordinates, double tool_length);
                void moveToDesirePosition_jacobian(Eigen::MatrixXd T,  geometry_msgs::Point nextPose, std::vector<double> des_polarCoordinates, std::vector<double> polarCoordinates, double tool_length);
                std::vector<double> getPolarCoordinates(const geometry_msgs::Point& fulcrum_point_current, double tool_length);
                geometry_msgs::Point generateNextPoint(const geometry_msgs::Point& desPose, const Eigen::MatrixXd& T_TTP);
                void computeRobotCinematic(double L);
                void servoJointPose();

                FTResult compensateForce(const Eigen::Matrix4d& T_E, const Eigen::Vector3d& weight_base, const Eigen::Vector3d& tool_CoM, double res_xy,double res_z, FTSensor* ftSensor);
                Eigen::Vector3d computeTipForceControl(double Kf, std::vector<double> tipForces_base, Eigen::Vector3d v_cartesian);
                std::vector<double> forceControl(const double& Kf, std::vector<double> forces);
                //fulcro
                geometry_msgs::Point computeFulcrum(Eigen::MatrixXd E_T_Fp, Eigen::MatrixXd T_E);
                Vector3d P0;
                //time
                ros::Time getStartTime() const;

};