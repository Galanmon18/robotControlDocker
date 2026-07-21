#include <services/jacobianControl.hpp>

jacobianControl::jacobianControl(ur_script* urScript, UMA_trans* umaTf,
                       Init* initRobot, PIDController* composePID, selectTool* selectTool, fulcrum* fp, FTSensor* ftSensor,  tf::TransformListener* tf_listener, std::string t_prefix) : 
                       ur(urScript), tr(umaTf), init(initRobot), controlPosition(composePID), tool(selectTool), fulcrumEstimation(fp), ftSensor(ftSensor), listener(tf_listener){//lista de inicialización
    std::cout << "Entrando al constructor jacobianControl..." << std::endl;
    std::cout << t_prefix << " control build" << std::endl;
    prefix_in = t_prefix;
    // Publicacion topics
    ttp_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("pose_topic", 1000);
    te_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("effectorFinal_topic", 1000);
    vel_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("velocity_topic", 1000);
    std::cout << "Constructor jacobianControl completado" << std::endl;
}
jacobianControl::~jacobianControl(){
    std::cout <<"Leaving gently jacobianControl..."<< std::endl;
}

Eigen::MatrixXd jacobianControl::readTransform(std::string base,std::string tool0){
    Eigen::MatrixXd T(4,4);
    try {
        listener->waitForTransform(base, tool0, ros::Time(0), ros::Duration(1.0));
        listener->lookupTransform(base, tool0, ros::Time(0), tf_pose);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR_STREAM("TF Exception: " << ex.what());
        return T;  // ❗ Salimos: no seguimos con datos corruptos
    }
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

void jacobianControl::computeRobotCinematic(double L){
    if (prefix_in.empty()) {
        base_name = prefix_in + "base";
        tool_name = prefix_in + "tool0_controller";
        efector_name = prefix_in + "tool0";
    } else {
        base_name = prefix_in + "_base";
        tool_name = prefix_in + "_tool0_controller";
        efector_name = prefix_in + "_tool0";
    }
    T_E = readTransform(base_name, efector_name);
    std::cout << prefix_in << "T_E =[" << T_E << "];" << std::endl;
    T_TTP = readTransform(base_name, tool_name);
    std::cout << prefix_in << "T_TTP =[" << T_TTP << "];" << std::endl;
    //pub robot pose T_TTP and T_E
    for (int i = 0; i < T_TTP.rows(); ++i) {
        for (int j = 0; j < T_TTP.cols(); ++j) {
            pose_.data.push_back(T_TTP(j,i));
        }
    }
    ttp_pub_.publish(pose_);
    pose_.data.clear();
    for (int i = 0; i < T_E.rows(); ++i) {
        for (int j = 0; j < T_E.cols(); ++j) {
            poseE_.data.push_back(T_E(j,i));
        }
    }
    te_pub_.publish(poseE_);
    poseE_.data.clear();
}

int main(int argc, char **argv){
    //ros
    ros::init(argc, argv, "calibration");
    ros::NodeHandle nh_param("~"); // Nota: el uso de '~' para obtener los parámetros relativos al namespace del nodo
    ros::Publisher  ttp_pub_ = nh_param.advertise<std_msgs::Float64MultiArray>("pose_topic", 1000);
    ros::Publisher  te_pub_ = nh_param.advertise<std_msgs::Float64MultiArray>("effectorFinal_topic", 1000);
    ros::Rate rate(125);

    std::string prefix;
    double tool_length;
    nh_param.param<std::string>("prefix", prefix, "auto");
    nh_param.param<double>("tool_length", tool_length, 0.013);
    
    std::cout << "Creando ur_script..." << std::endl;
    ur_script ur(prefix);
    std::cout << "ur_script creado correctamente" << std::endl;
    
    std::cout << "Creando UMA_trans..." << std::endl;
    UMA_trans tr;
    std::cout << "UMA_trans creado correctamente" << std::endl;
    
    std::cout << "Creando Init..." << std::endl;
    Init init;
    std::cout << "Init creado correctamente" << std::endl;
    
    PIDController controlPosition(0,0,0,0.008,6);
    selectTool tool;
    fulcrum fulcrumEstimation;
    FTSensor ftSensor("192.168.1.00");
    tf::TransformListener tf_listener;
    ros::Duration(1.0).sleep(); 
    
    std::cout << "Todos los objetos creados, creando jacobianControl..." << std::endl;
    jacobianControl robot(&ur, &tr, &init, &controlPosition, &tool, &fulcrumEstimation, &ftSensor, &tf_listener,prefix);
    std::cout << "jacobianControl creado correctamente" << std::endl;

    //----------------
    std::vector<double> TCP = {0., 0, tool_length, 0., 0., 0.};
    std::cout << "3" << std::endl;
    ur.set_tcp(TCP);
    std::cout << "4" << std::endl;
    ros::Duration(1).sleep();
    std::cout << "vamos al while" << std::endl;
    // Variables globales para publicar
    /*std_msgs::Float64MultiArray pose_;
    std_msgs::Float64MultiArray poseE_;
    Eigen::MatrixXd T_E, T_TTP;
    Eigen::MatrixXd T = robot.readTransform("auto_base", "auto_tool0");*/
    while (ros::ok()){
        robot.computeRobotCinematic(tool_length);
        //std::cout << "..." << std::endl;
        ros::spinOnce();
        rate.sleep();
    }
}