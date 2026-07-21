// Nombre: ur_script.cpp
// Paquete: craneeal_usage
// Libreria: ur_script_lib

// Descripción:
// Código fuente de la libreria de funciones UR Script

#include "dependecies/ur_script.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h> 
#include <cmath> 
// Nombre: ur_script::ur_script
// Argumentos: -
// Devuelve: -
//
// Descripción:
// Constructor de la clase ur_script. Inicializa el publicador en el
// topic /ur_driver/URScript que se comunica con el robot
//ur_script::ur_script() : tf_listener(tf_buffer)//
ur_script::ur_script(const std::string& topic_prefix) : tf_listener(tf_buffer)
{
    //create topic name
    prefix = topic_prefix;
    std::string topic_name = "/ur_hardware_interface/script_command";
    //std::string topic_name = "/ur_hardware_interface/script_command";
    ROS_INFO_STREAM("topic_name");
    ROS_INFO_STREAM(topic_name);
    //teleop_step = DEFAULT_TELEOP_STEP;
    publisher = node_handle.advertise<std_msgs::String>(topic_name, 250);
    ros::Duration(1.0).sleep();

    // Permite cambiar Verbosity. Por defecto: info
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
        ros::console::notifyLoggerLevelsChanged();
}


// Nombre: ur_script::set_tcp
// Argumentos: xd -> posicion del tcp, 3 primeros datos x, y, z en m y 3 siguientes dx, dy, dz en rad
// Devuelve: -
//
// Descripción:
// Modifica la posición del TCP.
//
void ur_script::set_tcp(std::vector<double> xd)
{
    std_msgs::String msg;
    std::stringstream stream;

    stream << "set_tcp(p";
    stream << qd_to_string(xd);
    stream << ")";
    //stream << "set_tcp(p[0.,.0,.8,0.,0.,0.])";
    msg.data = stream.str();

    ROS_DEBUG("%s", msg.data.c_str());
    ROS_DEBUG("set_tcp enviado");

    publisher.publish(msg);
}


// Nombre: ur_script::freedrive
// Argumentos: 
// Devuelve: -
//
// Descripción:
// Libera al robot para su movimiento libre, debe ejecutarse en bucle.
//
void ur_script::freedrive()
{
    std_msgs::String msg;
    std::stringstream stream;

    stream << "freedrive_mode()";

    msg.data = stream.str();

    ROS_DEBUG("%s", msg.data.c_str());
    ROS_DEBUG("freedrive enviado");

    publisher.publish(msg);
}

// Nombre: ur_script::movej
// Argumentos: q -> configuracion de ejes [base,shoulder,elbow,wrist1,wrist2,wrist3] en radianes
//             blocking -> si vale true, espera que termine el movimeinto para continuar el programa
//             a -> aceleracion en rad/s2
//             v -> velocidad en rad/s
//             t -> tiempo de ejecucion en s (prioridad frente a aceleracion y velocidad)
//             r -> radio en m. Para encadenar entos
// Devuelve: -
//
// Descripción:
// Mueve el robot a una posicion linealmente en el joint-space
void ur_script::movej(std::vector<double> q, bool blooking, double a, double v, double t, double r)
{
    std_msgs::String msg;
    std::stringstream stream;

    stream << "movej(";
    stream << joints_to_string(q);
    stream << ",";
    stream << std::to_string(a);
    stream << ",";
    stream << std::to_string(v);
    stream << ",";
    stream << std::to_string(t);
    stream << ",";
    stream << std::to_string(r);
    stream << ")";

    msg.data = stream.str();

    ROS_DEBUG("%s", msg.data.c_str());
    ROS_DEBUG("movej enviado");

    publisher.publish(msg);

    if (blooking) //CAMBIAR TOPIC LECTURA EN LA FUNCION CHECK PARA TENER EN CUENTA EL CHECKEO, SINO NO FUNCIONA
    {
        while (!check_completion_q(q));
        ROS_INFO_STREAM("end of check_completion_q");
    }
}

// Nombre: ur_script::movel
// Argumentos: pose -> [x,y,z,rx,ry,rz] en metros y angle axis rotation
//             blocking -> si vale true, espera que termine el movimeinto para continuar el programa
//             a -> aceleracion en m/s2
//             v -> velocidad en m/s
//             t -> tiempo de ejecucion en s (prioridad frente a aceleracion y velocidad)
//             r -> radio en m. Para encadenar movimientos
// Devuelve: -
//
// Descripción:
// Mueve el robot a una posicion linealmente en el joint-space
//void ur_script::movel(std::vector<double> pose, bool blooking, double a, double v, double t, double r)
void ur_script::movel(geometry_msgs::Pose pose, bool blooking, double a, double v, double t, double r)
{
    std_msgs::String msg;
    std::stringstream stream;

    stream << "movel(";
    stream << geometry_pose_to_string(pose);
    stream << ",";
    stream << std::to_string(a);
    stream << ",";
    stream << std::to_string(v);
    stream << ",";
    stream << std::to_string(t);
    stream << ",";
    stream << std::to_string(r);
    stream << ")";

    msg.data = stream.str();

    ROS_DEBUG("%s", msg.data.c_str());
    ROS_INFO_STREAM("movel enviado");
    ROS_INFO_STREAM(geometry_pose_to_string(pose));

    publisher.publish(msg);

    if (blooking)
    {
        ROS_WARN("FOR MOVEL check_completion_pose NO implementado");
        //while (!check_completion_pose(pose))
        //     ;
    }
}

// Nombre: ur_script::movel_offset_tool
// Argumentos: offset_xyz -> [x,y,z] en metros
//             blocking -> si vale true, espera que termine el movimeinto para continuar el programa
//             a -> aceleracion en m/s2
//             v -> velocidad en m/s
//             t -> tiempo de ejecucion en s (prioridad frente a aceleracion y velocidad)
//             r -> radio en m. Para encadenar movimientos
// Devuelve: -
//
// Descripción:
// Mueve el robot a una posicion linealmente en el joint-space
void ur_script::movel_offset_tool(std::vector<double> offset_xyz, double a, double v, double t, double r)
{
    std_msgs::String msg;
    std::stringstream stream;

    stream << "movel(";
    stream << offset_tool(offset_xyz);
    stream << ",";
    stream << std::to_string(a);
    stream << ",";
    stream << std::to_string(v);
    stream << ",";
    stream << std::to_string(t);
    stream << ",";
    stream << std::to_string(r);
    stream << ")";

    msg.data = stream.str();

    ROS_DEBUG("%s", msg.data.c_str());
    ROS_DEBUG("movel_offset_tool enviado");

    publisher.publish(msg);
}

void ur_script::movel_offset_tool_rel(std::vector<double> joints, std::vector<double> offset_xyz, double a, double v, double t, double r)
{
    std_msgs::String msg;
    std::stringstream stream;

    stream << "movel(";
    stream << offset_tool_rel(offset_xyz, joints);
    stream << ",";
    stream << std::to_string(a);
    stream << ",";
    stream << std::to_string(v);
    stream << ",";
    stream << std::to_string(t);
    stream << ",";
    stream << std::to_string(r);
    stream << ")";

    msg.data = stream.str();

    ROS_DEBUG("%s", msg.data.c_str());
    ROS_DEBUG("movel_offset_tool_rel enviado");

    publisher.publish(msg);
}

void ur_script::movej_offset_tool_rel(std::vector<double> joints, std::vector<double> offset_xyz, double a, double v, double t, double r)
{
    std_msgs::String msg;
    std::stringstream stream;

    stream << "movej(";
    stream << offset_tool_rel(offset_xyz, joints);
    stream << ",";
    stream << std::to_string(a);
    stream << ",";
    stream << std::to_string(v);
    stream << ",";
    stream << std::to_string(t);
    stream << ",";
    stream << std::to_string(r);
    stream << ")";

    msg.data = stream.str();

    ROS_DEBUG("%s", msg.data.c_str());
    ROS_DEBUG("movej_offset_tool_rel enviado");

    publisher.publish(msg);
}

void ur_script::movej_offset_tool(std::vector<double> offset_xyz, double a, double v, double t, double r)
{
    std_msgs::String msg;
    std::stringstream stream;

    stream << "movej(";
    stream << offset_tool(offset_xyz);
    stream << ",";
    stream << std::to_string(a);
    stream << ",";
    stream << std::to_string(v);
    stream << ",";
    stream << std::to_string(t);
    stream << ",";
    stream << std::to_string(r);
    stream << ")";

    msg.data = stream.str();

    ROS_DEBUG("%s", msg.data.c_str());
    ROS_DEBUG("movej_offset_tool enviado");

    publisher.publish(msg);
}

void ur_script::movej_pose(std::vector<double> pose, bool blooking, double a, double v, double t, double r)
{
    std_msgs::String msg;
    std::stringstream stream;

    stream << "movej(";
    stream << pose_to_string(pose);
    stream << ",";
    stream << std::to_string(0.01);
    stream << ",";
    stream << std::to_string(0.01);
    stream << ",";
    stream << std::to_string(t);
    stream << ",";
    stream << std::to_string(r);
    stream << ")";

    msg.data = stream.str();

    ROS_DEBUG("%s", msg.data.c_str());
    ROS_DEBUG("movej enviado");

    publisher.publish(msg);

    if (blooking)
    {
        ROS_WARN("check_completion_pose NO implementado"); //ñ
        //while (!check_completion_pose(pose));
    }
}

// Nombre: ur_script::move_laparoscopy
// Argumentos: pose -> [x,y,z,rx,ry,rz] en metros y angle axis rotation

void ur_script::move_laparoscopy(geometry_msgs::Point posFinal, geometry_msgs::Point posRef, bool blooking, double a, double v, double t, double r)
{
    posRef.x=0.;
    posRef.y=0.;
    posRef.z=0.;
    posFinal.x =-0.3;
    posFinal.y =0.3;
    posFinal.z=0.3;
    double mod = sqrt(pow((posFinal.x-posRef.x),2)+pow((posFinal.y-posRef.y),2)+pow((posFinal.z-posRef.z),2));
    //std::vector<double> pose = {posFinal.x, posFinal.y, posFinal.z, 0, acos((posFinal.z-posRef.z)/mod),-acos((posFinal.x-posRef.x)/mod)};
    std::vector<double> pose = {posFinal.x, posFinal.y, posFinal.z, asin((posFinal.y-posRef.y)/mod)*-1, asin((posFinal.x-posRef.x)/mod),0.};
    std_msgs::String msg;
    std::stringstream stream;


    std::cout<<((posFinal.x-posRef.x)/mod)<<std::endl;
    std::cout<<((posFinal.y-posRef.y)/mod)<<std::endl;
    std::cout<<((posFinal.z-posRef.z)/mod)<<std::endl;
    std::cout<<mod<<std::endl;

    
    stream << "movej(";
    stream << pose_to_string(pose);
    stream << ",";
    stream << std::to_string(a);
    stream << ",";
    stream << std::to_string(v);
    stream << ",";
    stream << std::to_string(t);
    stream << ",";
    stream << std::to_string(r);
    stream << ")";
    msg.data = stream.str();

    ROS_DEBUG("%s", msg.data.c_str());
    ROS_DEBUG("movej enviado");

    publisher.publish(msg);

    if (blooking)
    {    
        //while (!check_completion_pose(pose))
        //    ;
    }
}

// Nombre: ur_script::servoj
// Argumentos: q -> configuracion de ejes [base,shoulder,elbow,wrist1,wrist2,wrist3] en radianes
//             a -> aceleracion en rad/s2. No utilizado por el driver
//             v -> velocidad en rad/s. No utilizado por el driver
//             t -> tiempo que el comando controla el robot (blocking). Por defecto 125 Hz (0.008 s)
//             la_t -> lookahead time (0.03, 0.2). Valor bajo -> Reaccion rapida /  Valor alto -> Reaccion lenta, previene overshoot
//             gain -> ganancia. Valor proporcional para la posicion objetivo. (100, 2000)
// Devuelve: -
//
// Descripción:
// Mueve el robot a una posicion linealmente en el joint-space. Si se desea una posicion y orientacion
// utilizar la cinematica inversa de MoveIt (poseIK), o la funcion ur_script::servoj_ik.
void ur_script::servoj(std::vector<double> q, double a, double v, double t, double la_t, double gain)
{
    std_msgs::String msg;
    std::stringstream stream;

    stream << "servoj(";
    stream << joints_to_string(q);
    stream << ",";
    stream << std::to_string(a);
    stream << ",";
    stream << std::to_string(v);
    stream << ",";
    stream << std::to_string(t);
    stream << ",";
    stream << std::to_string(la_t);
    stream << ",";
    stream << std::to_string(gain);
    stream << ")";

    msg.data = stream.str();

    ROS_DEBUG("%s", msg.data.c_str());
    ROS_DEBUG("servoj enviado");

    publisher.publish(msg);
}

// Nombre: ur_script::servoj_ik
// Argumentos: pose -> [x,y,z,rx,ry,rz] en metros y angle axis rotation
//             a -> aceleracion en rad/s2. No utilizado por el driver
//             v -> velocidad en rad/s. No utilizado por el driver
//             t -> tiempo que el comando controla el robot (blocking). Por defecto 125 Hz (0.008 s)
//             la_t -> lookahead time (0.03, 0.2). Valor bajo -> Reaccion rapida /  Valor alto -> Reaccion lenta, previene overshoot
//             gain -> ganancia. Valor proporcional para la posicion objetivo. (100, 2000)
// Devuelve: -
//
// Descripción:
// Mueve el robot a una posicion linealmente en el joint-space. Recibe una posicion y orientacion
// y la cinematica inversa la lleva a cabo
void ur_script::servoj_ik(std::vector<double> pose, double a, double v, double t, double la_t, double gain)
{
    std_msgs::String msg;
    std::stringstream stream;

    stream << "servoj(";
    stream << get_inverse_kin(pose);
    stream << ",";
    stream << std::to_string(a);
    stream << ",";
    stream << std::to_string(v);
    stream << ",";
    stream << std::to_string(t);
    stream << ",";
    stream << std::to_string(la_t);
    stream << ",";
    stream << std::to_string(gain);
    stream << ")";

    msg.data = stream.str();

    ROS_DEBUG("%s", msg.data.c_str());
    ROS_DEBUG("servoj_ik enviado");

    publisher.publish(msg);
}

void ur_script::servoj_offset_tool_rel(std::vector<double> joints, std::vector<double> offset_xyz, double a, double v, double t, double la_t, double gain)
{
    std_msgs::String msg;
    std::stringstream stream;

    stream << "servoj(get_inverse_kin(";
    stream << offset_tool_rel(offset_xyz, joints);
    stream << "),";
    stream << std::to_string(a);
    stream << ",";
    stream << std::to_string(v);
    stream << ",";
    stream << std::to_string(t);
    stream << ",";
    stream << std::to_string(la_t);
    stream << ",";
    stream << std::to_string(gain);
    stream << ")";

    msg.data = stream.str();

    ROS_DEBUG("%s", msg.data.c_str());
    ROS_DEBUG("servoj_ik enviado");

    publisher.publish(msg);
}
// Nombre: ur_script::joints_to_string
// Argumentos: vector de double
// Devuelve: string con contenido del vector
//
// Descripción:
// Crea un string con el contenido del vector que se pasa como argumento.
// Se espera una configuracion de ejes.
std::string ur_script::joints_to_string(std::vector<double> vector)
{
    std::string vector_string;

    vector_string.push_back('[');

    for (size_t i = 0; i < vector.size(); i++)
    {
        vector_string.append(std::to_string(vector.at(i)));
        if (i == (vector.size() - 1))
            vector_string.push_back(']');
        else
            vector_string.push_back(',');
    }

    return vector_string;
}

// Nombre: ur_script::pose_to_string
// Argumentos: vector de double
// Devuelve: string con contenido del vector
//
// Descripción:
// Crea un string con el contenido del vector que se pasa como argumento.
// Se espera una posicion y orientacion.
// IMPORTANTE. La orientación se debe pasar como axis-angle notation
// y NO como cuaternio o angulos de Euler.
std::string ur_script::pose_to_string(std::vector<double> vector)
{
    // Convertimos angulos de Euler en notacion axis-angle
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! //
    // TERMINAR //

    std::string vector_string;

    vector_string.append("p[");

    for (size_t i = 0; i < vector.size(); i++)
    {
        vector_string.append(std::to_string(vector.at(i)));
        if (i == (vector.size() - 1))
            vector_string.push_back(']');
        else
            vector_string.push_back(',');
    }

    return vector_string;
}

std::string ur_script::geometry_pose_to_string(geometry_msgs::Pose pose)
{
    std::string pose_string;

    pose_string.append("p[");

    // Agregar posición (x, y, z)
    pose_string.append(std::to_string(pose.position.x) + ",");
    pose_string.append(std::to_string(pose.position.y) + ",");
    pose_string.append(std::to_string(pose.position.z) + ",");

    // Agregar orientación (qx, qy, qz, qw)
    pose_string.append(std::to_string(pose.orientation.x) + ",");
    pose_string.append(std::to_string(pose.orientation.y) + ",");
    pose_string.append(std::to_string(pose.orientation.z));

    // Cerrar con "]"
    pose_string.append("]");

    return pose_string;
}

// Nombre: ur_script::get_inverse_kin
// Argumentos: vector de double
// Devuelve: string con contenido del vector
//
// Descripción:
// Crea un string con el contenido del vector que se pasa como argumento.
// Se una posicion y orientación de la forma [x,y,z,rx,ry,rz] y se genera
// el comando que calcula su cinemática inversa
std::string ur_script::get_inverse_kin(std::vector<double> vector)
{
    std::string vector_string;

    vector_string.append("get_inverse_kin(p[");

    for (size_t i = 0; i < vector.size(); i++)
    {
        vector_string.append(std::to_string(vector.at(i)));
        if (i == (vector.size() - 1))
            vector_string.append("])");
        else
            vector_string.push_back(',');
    }

    return vector_string;
}

// Nombre: ur_script::offset_tool
// Argumentos: vector de double con offset en eje xyz
// Devuelve: string con contenido del vector
//
// Descripción:
// Devuelve una pose respecto a base-frame correspondiente a un
// offset recibido como argumento respecto al tool-frame
std::string ur_script::offset_tool(std::vector<double> vector)
{
    std::string vector_string;

    vector_string.append("pose_trans(get_forward_kin(),p[");

    for (size_t i = 0; i < vector.size(); i++)
    {
        vector_string.append(std::to_string(vector.at(i)));
        if (i == (vector.size() - 1))
            vector_string.append(",0,0,0])");
        else
            vector_string.push_back(',');
    }

    return vector_string;
}


std::string ur_script::offset_tool_rel(std::vector<double> offset, std::vector<double> joints)
{
    std::string vector_string;

    vector_string.append("pose_trans(");
    vector_string.append(pose_to_string({-0.29515, -0.11235, 0.48090, 2.2214, 0.0000, -2.2214}));
    vector_string.append(",p[");

    for (size_t i = 0; i < offset.size(); i++)
    {
        vector_string.append(std::to_string(offset.at(i)));
        if (i == (offset.size() - 1))
            vector_string.append(",0,0,0])");
        else
            vector_string.push_back(',');
    }

    return vector_string;
}

// Nombre: ur_script::set_var
// Argumentos:
// Devuelve:
//
// Descripción:

void ur_script::set_var()
{
    std_msgs::String msg;
    std::stringstream stream;

    stream << "global adios = pose_trans(get_forward_kin(), p[0,0,0.1,0,0,0])";

    msg.data = stream.str();

    ROS_DEBUG("%s", msg.data.c_str());
    ROS_DEBUG("set_var enviado");

    publisher.publish(msg);
}

// Nombre: ur_script::check_completion_q
// Argumentos: vector objetivo (configuracion de ejes)
// Devuelve: true o false si esta en posicion
//
// Descripción:
// Comprueba si el robot se encuentra en la posicion que se pasa
// como argumento. La tolerancia se define en Q_CHECK_COMPLETION_TOL
// Valido para movimientos movimientos relativamente grandes, ya que
// la funcion ros::topic::waitForMessage requiere tiempo en crear y
// destruir el suscriptor. (~200 ms)
bool ur_script::check_completion_q(std::vector<double> goal)
{
    sensor_msgs::JointState msg;
    boost::shared_ptr<sensor_msgs::JointState const> sharedPtr;
    
    double aux = goal.at(2);

    goal.at(2) = goal.at(0);
    goal.at(0) = aux;

    // Esperamos a recibir el ultimo mensaje del topic
    std::string topic_read = "/" + prefix + "/joint_states";
    sharedPtr = ros::topic::waitForMessage<sensor_msgs::JointState>(topic_read);
    msg = *sharedPtr;

    //std::cout<<msg<<std::endl;

    // Comprobamos eje a eje la posicion
    for (size_t i = 0; i < goal.size(); i++)
    {
        if ((msg.position.at(i) < (goal.at(i) - Q_CHECK_COMPLETION_TOL)) || (msg.position.at(i) > (goal.at(i) + Q_CHECK_COMPLETION_TOL)))
        {
            ROS_DEBUG_STREAM("Joint " << i << " out of tolerance (" << msg.position.at(i) << ")");
            return false;
        }
    }    
    ROS_DEBUG_STREAM("Ok");
    return true;
}

// Nombre: ur_script::check_completion_pose
// Argumentos: vector objetivo (pose respecto a /base)
// Devuelve: true o false si esta en posicion
// ñ
// Descripción:
// Comprueba si el robot se encuentra en la posicion que se pasa
// como argumento. La tolerancia se define en L_CHECK_COMPLETION_TOL
// para la posicion y Q_CHECK_COMPLETION_TOL para la orientacion
bool ur_script::check_completion_pose(std::vector<double> goal)
{
    geometry_msgs::TransformStamped pose;

    // Obtenemos ultima posicion
    if (prefix.empty()) {
        base_name = prefix + "base";
        tool_name = prefix + "tool0_controller";
    } else {
        base_name = prefix + "_base";
        tool_name = prefix + "_tool0_controller";
    }
    //pose = tf_buffer.lookupTransform("base", "tool0_controller", ros::Time(0));
    pose = tf_buffer.lookupTransform(base_name,tool_name, ros::Time(0));

    // Convertimos cuaternio a angulos de Euler
    tf2::Quaternion quaternion;
    double roll, pitch, yaw;

    quaternion.setX(pose.transform.rotation.x);
    quaternion.setY(pose.transform.rotation.y);
    quaternion.setZ(pose.transform.rotation.z);
    quaternion.setW(pose.transform.rotation.w);
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    // Comprobamos posicion y orientacion: [x,y,z,roll,pitch,yaw]
    for (size_t i = 0; i < goal.size(); i++)
    {
        double comp;

        switch (i)
        {
        case 0:
            comp = pose.transform.translation.x;
            break;
        case 1:
            comp = pose.transform.translation.y;
            break;
        case 2:
            comp = pose.transform.translation.z;
            break;
        case 3:
            comp = roll;
            break;
        case 4:
            comp = pitch;
            break;
        case 5:
            comp = yaw;
            break;
        default:
            break;
        }

        if (i < 3) // [x.y,z]
        {

            if ((comp < (goal.at(i) - L_CHECK_COMPLETION_TOL)) || (comp > (goal.at(i) + L_CHECK_COMPLETION_TOL)))
            {
                ROS_DEBUG_STREAM("Position " << i << " out of tolerance (" << comp << ")");
                return false;
            }
        }

        else // [roll,pitch,yaw]
        {
            if ((comp < (goal.at(i) - Q_CHECK_COMPLETION_TOL)) || (comp > (goal.at(i) + Q_CHECK_COMPLETION_TOL)))
            {
                ROS_DEBUG_STREAM("Orientation " << i << " out of tolerance (" << comp << ")");
                return false;
            }
        }
    }
    ROS_DEBUG_STREAM("Ok");
    return true;
}

// Nombre: ur_script::print_status
// Argumentos: -
// Devuelve: -
//
// Descripción:
// Saca por pantalla joints y pose
void ur_script::print_status()
{
    // Obtenemos ultima posicion
    geometry_msgs::TransformStamped pose;
    pose = tf_buffer.lookupTransform("base", "tool0_controller", ros::Time(0));

    // Convertimos cuaternio a angulos de Euler
    tf2::Quaternion quaternion;
    quaternion.setX(pose.transform.rotation.x);
    quaternion.setY(pose.transform.rotation.y);
    quaternion.setZ(pose.transform.rotation.z);
    quaternion.setW(pose.transform.rotation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    ROS_INFO_STREAM_THROTTLE(1, "RPY: [" << roll << " " << pitch << " " << yaw << "]");
}

// Nombre: ur_script::quaternion_to_aa
// Argumentos: Cuaternio (geometry_msgs::Quaternion)
// Devuelve: Orientacion en forma de angle axis rotation
//
// Descripción:
// Convierte la orientacion de cuaternios a angle
// axis rotation para poder utilizarlo en comandos de
// movimeinto.
std::vector<double> ur_script::quaternion_to_aa(geometry_msgs::Pose quaternion)
{
    Eigen::Quaternion<double> eigen_quaternion(quaternion.orientation.w,
                                               quaternion.orientation.x,
                                               quaternion.orientation.y,
                                               quaternion.orientation.z);
    eigen_quaternion.normalize();
    Eigen::Matrix3d rotation_matrix = eigen_quaternion.toRotationMatrix();
    Eigen::Vector3d rpy = rotation_matrix.eulerAngles(2, 1, 0);
    Eigen::AngleAxis<double> angle_axis(rotation_matrix);

    std::cout << rpy << std::endl;

    // TF2 library
    // Convertimos cuaternio a angulos de Euler
    tf2::Quaternion tf2_quaternion;
    tf2_quaternion.setX(quaternion.orientation.x);
    tf2_quaternion.setY(quaternion.orientation.y);
    tf2_quaternion.setZ(quaternion.orientation.z);
    tf2_quaternion.setW(quaternion.orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2_quaternion).getRPY(roll, pitch, yaw);
    std::vector<double> aa {yaw, pitch, roll};
    ROS_INFO_STREAM_THROTTLE(1, "RPY: [" << roll << " " << pitch << " " << yaw << "]");

    return {aa};
}

// Nombre: ur_script::speedl
// Argumentos: xd -> velocidad herramienta [m/s Y rad] -> [x,y,z,rx,ry,rz]
//             a -> aceleración de la posicion de la herramienta [m/s2]
//             t -> tiempo [s] de ejecucion
//             aRot -> aceleracion herramienta [rad/s2]
// Devuelve: -
//
// Descripción:
// Accelerate linearly in Cartesian space and continue with constant tool
// speed. The time t is optional; if provided the function will return after
// time t, regardless of the target speed has been reached. If the time t is
// not provided, the function will return when the target speed is reached.
// IMPORTANTE. VELOCIDADES EN METROS/S, NO MILIMETROS/S
void ur_script::speedl(std::vector<double> xd, double a, double t)
{
    std_msgs::String msg;
    std::stringstream stream;

    if (t != 0.0) // Con consigna de tiempo
    {
        stream << "speedl(";
        stream << qd_to_string(xd);
        stream << ",";
        stream << std::to_string(a);
        stream << ",";
        stream << std::to_string(t);
        stream << ")";
    }

    else // Sin consigna de tiempo
    {
        stream << "speedl(";
        stream << qd_to_string(xd);
        stream << ",";
        stream << std::to_string(a);
        stream << ")";
    }

    msg.data = stream.str();

    ROS_DEBUG("%s", msg.data.c_str());
    ROS_DEBUG("speedl enviado"); // "ROS_DEBUG" porque este comando se utilza a alta frecuencia
    //ROS_INFO_STREAM(msg.data);

    publisher.publish(msg);
}

// Nombre: ur_script::qd_to_string
// Argumentos: vector de double
// Devuelve: string con contenido del vector
//
// Descripción:
// Crea un string con el contenido del vector que se pasa como argumento.
// Se espera velocidades [x,y,z,rx,ry,rz]
std::string ur_script::qd_to_string(std::vector<double> qd)
{
    std::string qd_string;

    qd_string.push_back('[');

    for (size_t i = 0; i < qd.size(); i++)
    {
        qd_string.append(std::to_string(qd.at(i)));
        if (i == (qd.size() - 1))
            qd_string.push_back(']');
        else
            qd_string.push_back(',');
    }

    return qd_string;
}

// Nombre: ur_script::stopl
// Argumentos: a -> deceleracion de la herramienta [m/s2]
// Devuelve: -
//
// Descripción:
// Decelera hasta velocidad cero con una aceleracion que se pasa
// por argumento.
void ur_script::stopl(double a)
{
    std_msgs::String msg;
    std::stringstream stream;

    stream << "stopl(";
    stream << std::to_string(a);
    stream << ")";

    msg.data = stream.str();

    ROS_DEBUG("%s", msg.data.c_str());
    ROS_DEBUG("stopl enviado");

    publisher.publish(msg);
}
/*
void ur_script::teleop_callback(const std_msgs::Int8ConstPtr &msg)
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    switch (msg->data)
    {
    case KEYCODE_L: // -X
        x -= teleop_step;
        ROS_INFO_STREAM("Cmd: -X");
        break;
    case KEYCODE_R: // +X
        x += teleop_step;
        ROS_INFO_STREAM("Cmd: +X");
        break;
    case KEYCODE_U: // +Y
        y += teleop_step;
        ROS_INFO_STREAM("Cmd: +Y");
        break;
    case KEYCODE_D: // -Y
        y -= teleop_step;
        ROS_INFO_STREAM("Cmd: -Y");
        break;
    case KEYCODE_RE_PAG: // +Z
        z += teleop_step;
        ROS_INFO_STREAM("Cmd: +Z");
        break;
    case KEYCODE_AV_PAG: // -Z
        z -= teleop_step;
        ROS_INFO_STREAM("Cmd: -Z");
        break;
    case KEYCODE_PLUS: // Step +0.2mm
        teleop_step += 0.2;
        ROS_INFO_STREAM("Cmd: +0.2mm | Paso actual: " << teleop_step << "mm");
        break;
    case KEYCODE_MINUS: // Step -0.2
        if (teleop_step <= 0.2)
        {
            ROS_WARN_STREAM("Paso minimo alcanzado. Paso actual: 0.2mm");
        }
        else
        {
            teleop_step -= 0.2;
            ROS_INFO_STREAM("Cmd: -0.2mm | Paso actual: " << teleop_step << "mm");
        }
        break;
    }

    if (x != 0.0 || y != 0.0 || z != 0.0) // Comando de movimiento
    {
        movel_offset_tool({x / 1000.0, y / 1000.0, z / 1000.0}, 10.0e-3, 10.0e-3);
    }
}*/