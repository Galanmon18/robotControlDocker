#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <boost/foreach.hpp>
#include <fstream>
#include <iostream>
#include <glob.h>
#include <map>

#define foreach BOOST_FOREACH

std::string getHeader(const std::string& topic) {
    if (topic == "/auto/effectorFinal_force" || topic == "/auto/testForces/tissue_force" || topic == "/auto/MyJacobianControl/tissue_force") {
        return "Fx,Fy,Fz,Tx,Ty,Tz,time";
    } else if (topic == "/auto/velocity_topic") {
        return "dX,dY,dZ,dRX,dRY,dRZ,time";
    } else if (topic == "/auto/fulcrum") {
        return "X,Y,Z,time";
    } else if (topic =="/auto/robotbase_force") {
        return "Fx,Fy,Fz,time";
    } else if (topic =="/darel/abdomen_force_topic" || topic =="/darel/abdomen_effectorForce_topic") {
        return "Fx,Fy,Fz,Tx,Ty,Tz,time";
    } else if (topic == "/ur3e/rtde/force") {
        return "linear_x,linear_y,linear_z,angular_x,angular_y,angular_z,time";
    } else if (topic == "/tissueForce_topic") {
        return "F,time";
    } else if (topic == "/auto/dist_velocity_topic") {
        return "dist,vel,time";
    } else if (topic == "/auto/polar/current") {
        return "alpha,beta,rho,time";
    } else if (topic == "/auto/polar/des") {
        return "alpha,beta,rho,time";
    } else if (topic == "/auto/polar/delta") {
        return "alpha,beta,rho,time";
    } else if (topic == "/auto/pose_topic" || topic == "/darel/pose_topic" || topic == "/auto/effectorFinal_topic") {
        std::string header;
        for (int i = 1; i <= 16; ++i) {
            header += "V" + std::to_string(i);
            if (i != 16) header += ",";
        }
        header += ",time";
        return header;
    }
    return "";
}

int expectedSize(const std::string& topic) {
    if (topic == "/auto/effectorFinal_force" || topic == "/auto/testForces/tissue_force" || topic == "/auto/MyJacobianControl/tissue_force") return 6;
    if (topic == "/darel/abdomen_force_topic" || topic == "/darel/abdomen_effectorForce_topic") return 6;
    if (topic == "/auto/velocity_topic") return 6;
    if (topic == "/auto/fulcrum" || topic =="/auto/robotbase_force") return 3;
    if (topic == "/auto/pose_topic" || topic == "/auto/effectorFinal_topic") return 16;
    if (topic == "/darel/pose_topic") return 16;
    if (topic == "/ur3e/rtde/force") return 6; // 3 lineales + 3 angulares
    if (topic == "/tissueForce_topic") return 1;
    if (topic == "/auto/dist_velocity_topic") return 2;
    if (topic == "/auto/polar/current" || topic == "/auto/polar/des" || topic == "/auto/polar/delta") return 3;
    return 0;
}

std::string sanitizeFileName(const std::string& topic) {
    std::string base = topic;
    std::replace(base.begin(), base.end(), '/', '_');  // Reemplaza '/' por '_'
    if (!base.empty() && base[0] == '_') base = base.substr(1); // quita '_' inicial si existe
    return base + ".csv";
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "bag_to_csv_all_topics");

    if (argc < 2) {
        std::cerr << "Uso: " << argv[0] << " <directorio_de_bags>" << std::endl;
        return 1;
    }

    std::string bag_directory = argv[1];
    std::string pattern = bag_directory + "/*.bag";

    glob_t glob_result;
    if (glob(pattern.c_str(), GLOB_ERR, nullptr, &glob_result) != 0) {
        std::cerr << "Error al leer el directorio: " << bag_directory << std::endl;
        return 1;
    }

    std::vector<std::string> topics = {
        "/auto/effectorFinal_force",
        "/auto/robotbase_force",
        "/auto/testForces/tissue_force",
        "/auto/MyJacobianControl/tissue_force",
        "/auto/velocity_topic",
        "/auto/pose_topic",
        "/darel/pose_topic",
        "/darel/abdomen_effectorForce_topic",
        "/darel/abdomen_force_topic",
        "/auto/effectorFinal_topic",
        "/auto/fulcrum",
        "/ur3e/rtde/force",
        "/tissueForce_topic",
        "/auto/dist_velocity_topic",
        "/auto/polar/current",
        "/auto/polar/des",
        "/auto/polar/delta"
    };

    for (size_t i = 0; i < glob_result.gl_pathc; ++i) {
        std::string bag_file = glob_result.gl_pathv[i];
        rosbag::Bag bag;
        try {
            bag.open(bag_file, rosbag::bagmode::Read);
        } catch (rosbag::BagIOException &e) {
            std::cerr << "Error al abrir el archivo: " << bag_file << std::endl;
            continue;
        }

        rosbag::View view(bag, rosbag::TopicQuery(topics));

        std::map<std::string, std::ofstream> csv_files;
        std::map<std::string, bool> header_written;
        ros::Time start_time;
        bool first = true;

        foreach (rosbag::MessageInstance const m, view) {
            std::string topic = m.getTopic();
            
            if (first) {
                start_time = m.getTime();
                first = false;
            }

            double rel_time = (m.getTime() - start_time).toSec();

            // Abrir csv si no existe
            if (csv_files.find(topic) == csv_files.end()) {
                std::string output_path = bag_file.substr(0, bag_file.find_last_of("/\\") + 1) + sanitizeFileName(topic);
                csv_files[topic].open(output_path);
                if (!csv_files[topic]) {
                    std::cerr << "No se pudo abrir el archivo CSV para: " << topic << std::endl;
                    continue;
                }
                header_written[topic] = false;
            }

            std::ofstream& file = csv_files[topic];

            if (!header_written[topic]) {
                file << getHeader(topic) << "\n";
                header_written[topic] = true;
            }

            // Procesar según el tipo de mensaje
            if (topic == "/ur3e/rtde/force") {
                // Procesar geometry_msgs/Twist
                geometry_msgs::Twist::ConstPtr twist_msg = m.instantiate<geometry_msgs::Twist>();
                if (twist_msg) {
                    file << twist_msg->linear.x << ","
                         << twist_msg->linear.y << ","
                         << twist_msg->linear.z << ","
                         << twist_msg->angular.x << ","
                         << twist_msg->angular.y << ","
                         << twist_msg->angular.z << ","
                         << rel_time << "\n";
                }
            } else if (topic == "/tissueForce_topic") {
                // Procesar std_msgs::Float64 (caso especial)
                std_msgs::Float64::ConstPtr float_msg = m.instantiate<std_msgs::Float64>();
                if (float_msg) {
                    file << float_msg->data << "," << rel_time << "\n";
                }
            } else {
                // Procesar std_msgs::Float64MultiArray (comportamiento original)
                std_msgs::Float64MultiArray::ConstPtr msg = m.instantiate<std_msgs::Float64MultiArray>();
                if (!msg || msg->data.size() < expectedSize(topic)) continue;

                // Escribir datos + tiempo (al final)
                for (size_t j = 0; j < expectedSize(topic); ++j) {
                    file << msg->data[j];
                    file << ((j < expectedSize(topic) - 1) ? "," : "");
                }
                file << "," << rel_time << "\n";
            }
        }

        for (auto& pair : csv_files) {
            pair.second.close();
            std::cout << "Archivo guardado: " << pair.first << std::endl;
        }

        bag.close();
    }

    globfree(&glob_result);
    return 0;
}
