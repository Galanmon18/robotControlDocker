#include <ros/ros.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <array>
#include <sstream>
#include <iomanip>

int createServerSocket(int port)
{
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(port);
    bind(server_fd, (sockaddr*)&addr, sizeof(addr));
    listen(server_fd, 1);

    ROS_INFO("Esperando conexion del robot en puerto %d...", port);
    sockaddr_in client_addr{};
    socklen_t len = sizeof(client_addr);
    int conn_fd = accept(server_fd, (sockaddr*)&client_addr, &len);
    ROS_INFO("Robot conectado: %s", inet_ntoa(client_addr.sin_addr));

    close(server_fd);
    return conn_fd;
}

void sendPose(int conn_fd, const std::array<double, 6>& pose)
{
    std::ostringstream ss;
    ss.imbue(std::locale("C"));
    ss << std::fixed << std::setprecision(6);
    ss << '(';
    for (int i = 0; i < 6; ++i)
    {
        ss << pose[i];
        if (i < 5) ss << ',';
    }
    ss << ")\n";
    std::string msg = ss.str();
    send(conn_fd, msg.c_str(), msg.size(), 0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur_realtime_control");
    ros::NodeHandle nh("~");
    int port, rate_hz;
    nh.param<int>("tcp_port", port,    50000);
    nh.param<int>("rate",     rate_hz, 125);

    int conn_fd = createServerSocket(port);

    std::array<double, 6> pose_des = {0.34, 0.14, 0.17, 0.0, 3.14, 0.0};

    const double z_center    = 0.17;   // centro de la sinusoide
    const double z_amplitude = 0.05;   // ±5 cm
    const double freq_hz     = 0.2;    // 0.2 Hz → ciclo de 5 segundos

    ros::Rate rate(rate_hz);
    ros::Time t0 = ros::Time::now();

    while (ros::ok())
    {
        double t = (ros::Time::now() - t0).toSec();
        pose_des[2] = z_center + z_amplitude * std::sin(2.0 * M_PI * freq_hz * t);

        sendPose(conn_fd, pose_des);
        /*ROS_INFO_THROTTLE(1.0, "Enviando: %.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
            pose_des[0], pose_des[1], pose_des[2],
            pose_des[3], pose_des[4], pose_des[5]);*/

        ros::spinOnce();
        rate.sleep();
    }

    close(conn_fd);
    return 0;
}