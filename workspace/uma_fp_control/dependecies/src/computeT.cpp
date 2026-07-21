#include <dependecies/computeT.hpp>
#define PI 3.1416
//#include <tf/transform_listener.h>

Cinematic::Cinematic()//Constructor
{
	ROS_INFO_STREAM("---Cinematic---");
  fulcrum_point_goal.x = 0;
  fulcrum_point_goal.y = 0;
  fulcrum_point_goal.z = 0;
  alpha = 0;
  beta = 0;
}

Cinematic::~Cinematic(){
    ROS_INFO_STREAM("Leaving gently...");
}

void Cinematic::computeCinematicTf(Eigen::MatrixXd T_dest, Eigen::MatrixXd T_effector, Eigen::MatrixXd T_TipTopPosition, geometry_msgs::Point PositionStitches,
                geometry_msgs::Point last_PositionStitches, std::string topic_prefix,
                Eigen::MatrixXd E_T_M, Eigen::MatrixXd E_T_Fp, Eigen::MatrixXd T_FpIn,
                double p_estimado, bool flagFp,bool flagForce,UMA_trans* tr,PoseDesJM* Pdest){

  Eigen::MatrixXd dd(4,4);
  Eigen::MatrixXd local_E_T_Fp = E_T_Fp;
  if (flagFp){
    T_TTP = T_effector*E_T_M;
    T_Fp = T_effector *E_T_Fp;
  }
  else{
    T_Fp = T_FpIn;
    T_TTP = T_TipTopPosition;
  }
  if((last_PositionStitches != PositionStitches) || flagForce){
    TTP_dest = tr->desp({PositionStitches.x - T_TTP(0,3), PositionStitches.y - T_TTP(1,3), PositionStitches.z - T_TTP(2,3)}) * T_TTP;
    if(p_estimado > 0){
      TTP_dest_orientado = Pdest->computePoseDes(TTP_dest,T_Fp);
    } else {
      TTP_dest_orientado = TTP_dest;
    }
  } else if (PositionStitches.x == 0 && PositionStitches.y == 0 && PositionStitches.z == 0){
    TTP_dest_orientado = T_TTP;
  }
  else{
    TTP_dest_orientado = T_dest;//T_TTP;
  }
}

void Cinematic::computePivotPose(Eigen::MatrixXd T_dest, Eigen::MatrixXd T_effector, Eigen::MatrixXd T_TTP, Eigen::MatrixXd T_Fp, geometry_msgs::Point PositionStitches,double p_estimado,UMA_trans* tr,PoseDesJM* Pdest){
  TTP_dest = tr->desp({PositionStitches.x - T_TTP(0,3), PositionStitches.y - T_TTP(1,3), PositionStitches.z - T_TTP(2,3)}) * T_TTP;
  if(p_estimado > 0){
    TTP_dest_orientado = Pdest->computePoseDes(TTP_dest,T_Fp);
  } else {
    TTP_dest_orientado = TTP_dest;
  }
}

void Cinematic::computeCinematicAlphaBeta(Eigen::MatrixXd T_dest, Eigen::MatrixXd T_effector, Eigen::MatrixXd T_TipTopPosition, geometry_msgs::Point PositionStitches,
                geometry_msgs::Point last_PositionStitches, Eigen::MatrixXd E_T_M, Eigen::MatrixXd E_T_Fp, Eigen::MatrixXd T_FpIn,
                double p_estimado,double tool_length, bool flagFp, UMA_trans* tr){
  Eigen::MatrixXd dd(4,4);
  Eigen::Vector4d APCurrentRot;
  Eigen::Vector4d APGoalRot;
  Eigen::MatrixXd local_E_T_Fp = E_T_Fp;
  if (flagFp){
    T_TTP = T_effector*E_T_M;
    T_Fp = T_effector *E_T_Fp;
  }
  else{
    T_Fp = T_FpIn;
    //T_TTP = T_TipTopPosition;
  }
  if((last_PositionStitches != PositionStitches)){
    TTP_dest = tr->desp({PositionStitches.x - T_TTP(0,3), PositionStitches.y - T_TTP(1,3), PositionStitches.z - T_TTP(2,3)}) * T_TTP;
    T_Fp = tr->desp({T_Fp(0,3), T_Fp(1,3), T_Fp(2,3)});
    std::cout << "T_TTP->"<< T_TTP << std::endl;
    std::cout << "T_Fp->"<< T_Fp << std::endl;
    std::cout << "TTP_dest->"<< TTP_dest << std::endl;
    if(p_estimado > 0){
      //Get current position from Fulcrum
      fulcrum_point_current.x = T_TTP(0,3) - T_Fp(0,3);
      fulcrum_point_current.y = T_TTP(1,3) - T_Fp(1,3);
      fulcrum_point_current.z = T_TTP(2,3) - T_Fp(2,3);
      std::cout << "fulcrum_point_current->"<< fulcrum_point_current << std::endl;
      eigen_fulcrum_point_current << fulcrum_point_current.x, fulcrum_point_current.y, fulcrum_point_current.z,1;
      //Get Goal position from Fulcrum
      fulcrum_point_goal.x = TTP_dest(0,3) - T_Fp(0,3);
      fulcrum_point_goal.y = TTP_dest(1,3) - T_Fp(1,3);
      fulcrum_point_goal.z = TTP_dest(2,3) - T_Fp(2,3);
      std::cout << "fulcrum_point_goal->"<< fulcrum_point_goal << std::endl;
      eigen_fulcrum_point_goal << fulcrum_point_goal.x, fulcrum_point_goal.y, fulcrum_point_goal.z,1;
      //estiamte current alpha and beta
      alpha_current = atan2(-fulcrum_point_current.y,-fulcrum_point_current.x);
      if (abs(alpha_current) > PI) alpha_current -= PI;
      if ((PI - 0.001) < fabs(alpha_current) && fabs(alpha_current) < (PI + 0.001)) {
          alpha_current = 0;
      } else if (fabs(alpha_current ) > PI) {
          alpha_current = alpha_current - PI;
      }
      beta_current = atan2(-fulcrum_point_current.z,sqrt(fulcrum_point_current.x*fulcrum_point_current.x+fulcrum_point_current.y*fulcrum_point_current.y));
      if ((PI - 0.001) < fabs(beta_current) && fabs(beta_current) < (PI + 0.001)) {
          beta_current = 0;
      } else if (fabs(beta_current) > PI) {
          beta_current = beta_current - PI;
      }
      std::cout << "current alpha-Beta->"<< alpha_current<<", "<<beta_current<< std::endl;
      rho = tool_length - sqrt(fulcrum_point_current.x * fulcrum_point_current.x +
                                         fulcrum_point_current.y * fulcrum_point_current.y +
                                         fulcrum_point_current.z * fulcrum_point_current.z);
      //estimate alpha
      double numerador_alpha = fulcrum_point_current.x * fulcrum_point_goal.y - 
                         fulcrum_point_current.y * fulcrum_point_goal.x;
      double denominador_alpha = fulcrum_point_current.x * fulcrum_point_goal.x + 
                                fulcrum_point_current.y * fulcrum_point_goal.y;
      double alpha_inc = atan2(numerador_alpha, denominador_alpha);
      // Sumo ángulos
      alpha = alpha_inc + alpha_current;
      //estimate beta
      APCurrentRot = tr->rotZ(-alpha_current) * eigen_fulcrum_point_current;
      APGoalRot = tr->rotZ(-alpha) * eigen_fulcrum_point_goal;
      double numerador_beta = APCurrentRot(0) * APGoalRot(2) - APCurrentRot(2) * APGoalRot(0); // Producto cruzado en ZX
      double denominador_beta = APCurrentRot(0) * APGoalRot(0) + APCurrentRot(2) * APGoalRot(2);
      double beta_inc = atan2(numerador_beta, denominador_beta);
      beta = beta_inc + beta_current;
      std::cout << "incrementos alpha-Beta-rho->"<< alpha<<" "<<beta<<" "<<rho << std::endl;
      //------------
      TTP_dest_orientado = T_Fp * tr->rotZ(alpha) * tr->rotY(-PI/2-beta) * tr->desp({0,0, tool_length-rho});
      std::cout << "TTP_dest_orientado alphaBeta->"<< TTP_dest_orientado << std::endl;
    } else {
      TTP_dest_orientado = TTP_dest;
    }
  } else if (PositionStitches.x == 0 && PositionStitches.y == 0 && PositionStitches.z == 0){
    TTP_dest_orientado = T_TTP;
  }
  else{
    TTP_dest_orientado = T_dest;
  }
}


void Cinematic::computeCinematicIncrement(Eigen::MatrixXd T_dest, Eigen::MatrixXd T_effector, Eigen::MatrixXd T_TipTopPosition, geometry_msgs::Point PositionStitches,
                geometry_msgs::Point last_PositionStitches, Eigen::MatrixXd E_T_M, Eigen::MatrixXd E_T_Fp, Eigen::MatrixXd T_FpIn,
                double p_estimado,double tool_length, bool flagFp, UMA_trans* tr) {
    Eigen::MatrixXd dd(4,4);
    Eigen::MatrixXd J_AH(3,3);
    if (flagFp) {
        T_TTP = T_effector * E_T_M;
        T_Fp = T_effector * E_T_Fp;
        T_Fp = tr->desp({T_Fp(0,3), T_Fp(1,3), T_Fp(2,3)});
        std::cout << "T_Fp 2 JACOBIANO->" << T_Fp << std::endl;
    } else {
        T_Fp = T_FpIn;
        T_TTP = T_TipTopPosition; // Use predefined tip position
    }

    // Check if position has changed
    if((last_PositionStitches != PositionStitches)) {
        // Calculate position difference
        std::cout << "T_TTP JACOBIANO->" << T_TTP << std::endl;
        TTP_dest = tr->desp({PositionStitches.x - T_TTP(0,3), PositionStitches.y - T_TTP(1,3), PositionStitches.z - T_TTP(2,3)}) * T_TTP;
        std::cout << "TTP_dest JACOBIANO->" << TTP_dest << std::endl;
        // Additional calculations if p_estimado is positive
        if (p_estimado > 0) {
            //Get current position from Fulcrum
            // Get current position from Fulcrum with inline rounding
            fulcrum_point_current.x = std::round((T_TTP(0,3) - T_Fp(0,3)) * 10000.0) / 10000.0;
            fulcrum_point_current.y = std::round((T_TTP(1,3) - T_Fp(1,3)) * 10000.0) / 10000.0;
            fulcrum_point_current.z = std::round((T_TTP(2,3) - T_Fp(2,3)) * 10000.0) / 10000.0;
            std::cout << "fulcrum_point_current->" << fulcrum_point_current << std::endl;
            // Get Goal position from Fulcrum with inline rounding
            fulcrum_point_goal.x = std::round((TTP_dest(0,3) - T_Fp(0,3)) * 10000.0) / 10000.0;
            fulcrum_point_goal.y = std::round((TTP_dest(1,3) - T_Fp(1,3)) * 10000.0) / 10000.0;
            fulcrum_point_goal.z = std::round((TTP_dest(2,3) - T_Fp(2,3)) * 10000.0) / 10000.0;
            //estiamte current alpha and beta
            alpha_current = atan2(-fulcrum_point_current.y,-fulcrum_point_current.x);
            if (abs(fulcrum_point_current.y) < 0.001) alpha_current = 0;

            beta_current = atan2(-fulcrum_point_current.z,sqrt(fulcrum_point_current.x*fulcrum_point_current.x+fulcrum_point_current.y*fulcrum_point_current.y));
            if (abs(beta_current) > PI) beta_current -= PI;

            alpha = alpha_current;
            beta = beta_current;
            rho = p_estimado;
            std::cout << "alpha->" << alpha * 180/PI << std::endl;
            std::cout << "beta->" << beta * 180/PI << std::endl;
            std::cout << "rho->" << rho << std::endl;
            
            for (int iter = 0; iter < MAX_ITER; ++iter) {
              double ca = cos(alpha), cb = cos(beta);
              double sa = sin(alpha), sb = sin(beta);
              J_AH << -(rho - tool_length) * sa * cb, -(rho - tool_length) * ca * sb, ca * cb,
                  (rho - tool_length) * ca * cb, -(rho - tool_length) * sa * sb, sa * cb,
                  0, (rho - tool_length) * cb, sb;
              // Pseudo-inverse of Jacobian
              Eigen::Matrix3d invJ_AH = J_AH.completeOrthogonalDecomposition().pseudoInverse();
              fulcrum_point_now.x = std::round(((rho - tool_length) * cos(alpha) * cos(beta)) * 10000.0) / 10000.0;
              fulcrum_point_now.y = std::round(((rho - tool_length) * sin(alpha) * cos(beta)) * 10000.0) / 10000.0;
              fulcrum_point_now.z = std::round(((rho - tool_length) *sin(beta)) * 10000.0) / 10000.0;
              fulcrum_difference_position << fulcrum_point_goal.x - fulcrum_point_now.x,
                  fulcrum_point_goal.y - fulcrum_point_now.y,
                  fulcrum_point_goal.z - fulcrum_point_now.z;
              // Check for convergence
              if (fulcrum_difference_position.norm() < TOLERANCE) {
                  std::cout << "Convergence achieved in " << iter + 1 << " iterations." << std::endl;
                  break;
              }
              // Solve for Delta_theta
              Eigen::Vector3d Delta_theta = invJ_AH * fulcrum_difference_position;
              // Update configuration variables
              alpha += Delta_theta(0);
              beta += Delta_theta(1);
              rho += Delta_theta(2);
              std::cout << "--------------" <<  std::endl;
              if (iter == MAX_ITER - 1) {
                  std::cout << "Warning: Convergence not achieved in the maximum number of iterations." << std::endl;
              }
            }
            double alpha_mod = fmod(alpha, 2*PI);
            if (alpha_mod < 0) {
              alpha =alpha_mod+ 2*PI;
            }
            double beta_mod = fmod(beta, 2*PI);
            if (beta_mod < 0) {
              beta =beta_mod+ 2*PI;
            }
            std::cout << "-------the end-------" <<  std::endl;
            std::cout << "alpha->" << alpha * 180/PI << std::endl;
            std::cout << "beta->" << beta * 180/PI << std::endl;
            std::cout << "rho->" << rho << std::endl;
            std::cout << "T_Fp->" << T_Fp << std::endl;
            std::cout << "PI->" << PI << std::endl;
            TTP_dest_orientado = T_Fp * tr->rotZ(alpha) * tr->rotY(-PI/2-beta) * tr->desp({0,0, tool_length-rho});
            std::cout << "TTP_dest_orientado JACOBIANO->" << TTP_dest_orientado << std::endl;
            std::cout << "**************" <<  std::endl;
        } else{
          TTP_dest_orientado = TTP_dest;
        }
    } else if (PositionStitches.x == 0 && PositionStitches.y == 0 && PositionStitches.z == 0){
      TTP_dest_orientado = T_dest;//T_TTP;
    }
    else{
      TTP_dest_orientado = T_dest;
    }
}

/* ESTO NO ME HACE FALTA PERO POR AHORA NO LO USAMOS
Eigen::VectorXd jacobianControl::velocityPropagation(const Eigen::VectorXd& delta_cartesian, const geometry_msgs::Point& P, const Eigen::MatrixXd& TA){
    // Convert geometry_msgs::Point to Eigen::Vector3d
    Eigen::Vector3d P_eigen(P.x, P.y, P.z);

    // Extract linear and angular velocity
    Eigen::Vector3d V_A = delta_cartesian.head<3>();  // First 3 elements
    Eigen::Vector3d W_A = delta_cartesian.tail<3>();  // Last 3 elements

    // Sacar omega de W
    Eigen::Vector3d Omega;
    Omega = TA * W_A;

    // Compute the propagated velocity
    Eigen::Vector3d V = V_A + W_A.cross(P_eigen);
    std::cout << " %%%%%%%%%%%%%%%%%%%%%%% "  << std::endl;
    std::cout << "fulcrum_point_goal: " << P_eigen << std::endl;
    std::cout << "W_A: " << W_A << std::endl;
    std::cout << " W_A.cross(P_eigen): " <<  W_A.cross(P_eigen) << std::endl;
    std::cout << " %%%%%%%%%%%%%%%%%%%%%%% "  << std::endl;

    // Concatenate V and W_A into a single vector
    Eigen::VectorXd Vpropagada(6);
    Vpropagada << V, W_A;

    return Vpropagada;
}*/