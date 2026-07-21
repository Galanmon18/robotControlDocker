#include <dependecies/computeError.hpp>

ErrorPose::ErrorPose()//Constructor
{
	ROS_INFO_STREAM("---Error---");
}

ErrorPose::~ErrorPose()
{
    ROS_INFO_STREAM("Leaving gently...");
}

std::vector<double> ErrorPose::computeError(Eigen::MatrixXd Dest, Eigen::MatrixXd Actual)
{
  Eigen::MatrixXd dR(4,4);
  //Destino
  RPYdest[0]=std::atan2(Dest(2,1),Dest(2,2));
  RPYdest[1]=std::atan2(-Dest(2,0),sqrt(Dest(0,0)*Dest(0,0)+Dest(1,0)*Dest(1,0)));
  RPYdest[2]=std::atan2(Dest(1,0),Dest(0,0));
  if (RPYdest[0]>(-0.001) && RPYdest[0]<(0.001))
  {
    RPYdest[0] = 0;
  }
  if (RPYdest[1]>(-0.001) && RPYdest[1]<(0.001))
  {
    RPYdest[1] = 0;
  }
  if (RPYdest[2]>(-0.001) && RPYdest[2]<(0.001))
  {
    RPYdest[2] = 0;
  }
  /*ROS_INFO_STREAM("RPY dest");
  ROS_INFO_STREAM(RPYdest[0]);
  ROS_INFO_STREAM(RPYdest[1]);
  ROS_INFO_STREAM(RPYdest[2]);*/
  //Actual
  RPY[0]=std::atan2(Actual(2,1),Actual(2,2));
  RPY[1]=std::atan2(-Actual(2,0),sqrt(pow(Actual(0,0),2)+pow(Actual(1,0),2)));
  RPY[2]=std::atan2(Actual(1,0),Actual(0,0));
  if ( RPY[0]>(-0.001) &&  RPY[0]<(0.001))
  {
     RPY[0] = 0;
  }
  if (RPY[1]>(-0.001) && RPY[1]<(0.001))
  {
    RPY[1] = 0;
  }
  if (RPY[2]>(-0.001) && RPY[2]<(0.001))
  {
    RPY[2] = 0;
  }
  /*ROS_INFO_STREAM("RPY");
  ROS_INFO_STREAM(RPY[0]);
  ROS_INFO_STREAM(RPY[1]);
  ROS_INFO_STREAM(RPY[2]);*/
  //error X TF
  /*error[0] = Dest(0,3) - Actual(0,3);
  error[0]*=1000;
  error[0]=round( error[0]);
  error[0]/=1000;
  error[0]=error[0];
  //error Y
  error[1] = Dest(1,3) - Actual(1,3);
  error[1]*=1000;
  error[1]=round(error[1]);
  error[1]/=1000;
  error[1]=error[1];
  //error Z
  error[2] = Dest(2,3) - Actual(2,3) ;
  error[2]*=1000;
  error[2]=round(error[2]);
  error[2]/=1000;*/

  //error X moveit
  error[0] = Dest(0,3) - Actual(0,3);
  error[0]*=1000;
  error[0]=round( error[0]);
  error[0]/=1000;
  error[0]=-error[0];
  //error Y
  error[1] = Dest(1,3) - Actual(1,3);
  error[1]*=1000;
  error[1]=round(error[1]);
  error[1]/=1000;
  error[1]=-error[1];
  //error Z
  error[2] = Dest(2,3) - Actual(2,3) ;
  error[2]*=1000;
  error[2]=round(error[2]);
  error[2]/=1000;

  //error roll
  /*if (RPY[0]<=0)
  {
    error[3] = fabs(RPYdest[0]) - fabs(RPY[0]);
    //error[3] = fabs(RPY[0]) - fabs(RPYdest[0]);
    ROS_INFO_STREAM("1");
    ROS_INFO_STREAM(error[3]);
  }
  if(RPY[0]>0)
  {
    //error[3] = fabs(RPYdest[0]) - fabs(RPY[0]);
    error[3] = fabs(RPY[0]) - fabs(RPYdest[0]);
    ROS_INFO_STREAM("2");
    ROS_INFO_STREAM(error[3]);
  }*/
  error[3] = (1*(RPYdest[0])) - (1*(RPY[0]));
  error[3]*=1000;
  error[3]=round(error[3]);
  error[3]/=1000;
  //error pitch
  error[4] = (RPYdest[1])-(RPY[1]);
  error[4]*=1000;
  error[4]=round(error[4]);
  error[4]/=1000;
  //error yaw
  error[5] = (RPYdest[2])-(RPY[2]);
  error[5]*=1000;
  error[5]=round(error[5]);
  error[5]/=1000;
  ROS_INFO_STREAM("error orientacion1");
  ROS_INFO_STREAM(error[3]);
  ROS_INFO_STREAM(error[4]);
  ROS_INFO_STREAM(error[5]);
  //matriz de error de orientacion
  dR = Dest.transpose() * Actual;
  error[3] = atan2(dR(2,1), dR(2,2));
  error[3]*=1000;
  error[3]=round(error[3]);
  error[3]/=1000;
  //error[3] = -error[3]; //CON TF
  /*if (error[3]>0){
    error[3]=error[3]-3,14159;
  } else{
    error[3]=error[3]+3,14159;
  }*/
  /*error[4] = atan2(-dR(2,0), sqrt(dR(0,0)*dR(0,0) + dR(1,0)*dR(1,0)));
  error[5] = atan2(dR(1,0), dR(0,0));*/

  error[4] = std::atan2(-dR(2,0),sqrt(pow(dR(0,0),2)+pow(dR(1,0),2)));
  error[4]*=1000;
  error[4]=round(error[4]);
  error[4]/=1000;
  //error yaw
  error[5] = std::atan2(dR(1,0),dR(0,0));
  error[5]*=1000;
  error[5]=round(error[5]);
  error[5]/=1000;

  /*ROS_INFO_STREAM("error orientacion");
  ROS_INFO_STREAM(error[3]);
  ROS_INFO_STREAM(error[4]);
  ROS_INFO_STREAM(error[5]);*/

  //error[5]=-error[5]; 
	return error;
}

std::vector<double> ErrorPose::computeErrorTf(const Eigen::MatrixXd& Dest, const Eigen::MatrixXd& Actual)
{
  // VALIDACIÓN CRÍTICA
  if (Dest.rows() != 4 || Dest.cols() != 4) {
      ROS_ERROR_STREAM("ERROR: Dest no es 4x4! Tamaño: " << Dest.rows() << "x" << Dest.cols());
      return error;
  }
  if (Actual.rows() != 4 || Actual.cols() != 4) {
      ROS_ERROR_STREAM("ERROR: Actual no es 4x4! Tamaño: " << Actual.rows() << "x" << Actual.cols());
      return error;
  }

  /*RPYdest[0]=std::atan2(Dest(2,1),Dest(2,2));
  RPYdest[1]=std::atan2(-Dest(2,0),sqrt(Dest(0,0)*Dest(0,0)+Dest(1,0)*Dest(1,0)));
  RPYdest[2]=std::atan2(Dest(1,0),Dest(0,0));
  if (RPYdest[0]>(-0.001) && RPYdest[0]<(0.001))
  {
    RPYdest[0] = 0;
  }
  if (RPYdest[1]>(-0.001) && RPYdest[1]<(0.001))
  {
    RPYdest[1] = 0;
  }
  if (RPYdest[2]>(-0.001) && RPYdest[2]<(0.001))
  {
    RPYdest[2] = 0;
  }
  //Actual
  RPY[0]=std::atan2(Actual(2,1),Actual(2,2));
  RPY[1]=std::atan2(-Actual(2,0),sqrt(pow(Actual(0,0),2)+pow(Actual(1,0),2)));
  RPY[2]=std::atan2(Actual(1,0),Actual(0,0));
  if ( RPY[0]>(-0.001) &&  RPY[0]<(0.001))
  {
     RPY[0] = 0;
  }
  if (RPY[1]>(-0.001) && RPY[1]<(0.001))
  {
    RPY[1] = 0;
  }
  if (RPY[2]>(-0.001) && RPY[2]<(0.001))
  {
    RPY[2] = 0;
  }*/

  Eigen::MatrixXd dR(4,4);
  std::vector<double> error_local={0,0,0,0,0,0};
  //error X TF
  error_local[0] = Dest(0,3) - Actual(0,3);
  error_local[0]*=1000;
  error_local[0]=round( error_local[0]);
  error_local[0]/=1000;
  error_local[0]=error_local[0];
  //error_local Y
  error_local[1] = Dest(1,3) - Actual(1,3);
  error_local[1]*=1000;
  error_local[1]=round(error_local[1]);
  error_local[1]/=1000;
  error_local[1]=error_local[1];
  //error_local Z
  error_local[2] = Dest(2,3) - Actual(2,3) ;
  error_local[2]*=1000;
  error_local[2]=round(error_local[2]);
  error_local[2]/=1000;

  try {
      dR = Actual.transpose() * Dest;
  } catch (const std::exception& e) {
      ROS_ERROR_STREAM("Error en multiplicación de matrices: " << e.what());
      return error;
  }
  //dR = Actual.transpose() * Dest; //Con dR se calcula la diferencia de orientacion en EJES moviles. Es decir, en TTP. Por eso luego tenemos que pasar esa diferencia a la base del robot.
  //printf("***4****>>>\n");
  error_local[3] = atan2(dR(2,1), dR(2,2));
  error_local[3]*=1000;
  error_local[3]=round(error_local[3]);
  error_local[3]/=1000;
  error_local[4] = std::atan2(-dR(2,0),sqrt(pow(dR(0,0),2)+pow(dR(1,0),2)));
  error_local[4]*=1000;
  error_local[4]=round(error_local[4]);
  error_local[4]/=1000;
  //error_local yaw
  error_local[5] = std::atan2(dR(1,0),dR(0,0)); 
  error_local[5]*=1000;
  error_local[5]=round(error_local[5]);
  error_local[5]/=1000;
  /*dR = Dest.transpose() * Actual; //Con dR se calcula la diferencia de orientacion en EJES moviles. Es decir, en TTP. Por eso luego tenemos que pasar esa diferencia a la base del robot.
  error[3] = atan2(dR(2,1), dR(2,2));
  error[3]*=1000;
  error[3]=round(error[3]);
  error[3]/=1000;
  error[3] = -error[3];
  error[4] = std::atan2(-dR(2,0),sqrt(pow(dR(0,0),2)+pow(dR(1,0),2)));
  error[4]*=1000;
  error[4]=round(error[4]);
  error[4]/=1000;
  error[4] = -error[4];
  //error yaw
  error[5] = std::atan2(dR(1,0),dR(0,0)); 
  error[5]*=1000;
  error[5]=round(error[5]);
  error[5]/=1000;*/

  /*error[3] = ((RPYdest[0])) - (1*(RPY[0]));
  error[3]*=1000;
  error[3]=round(error[3]);
  error[3]/=1000;
  //error pitch
  error[4] = (RPYdest[1])-(RPY[1]);
  error[4]*=1000;
  error[4]=round(error[4]);
  error[4]/=1000;
  //error yaw
  error[5] = (RPYdest[2])-(RPY[2]);
  error[5]*=1000;
  error[5]=round(error[5]);
  error[5]/=1000;*/

	return error_local;
}