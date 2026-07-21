#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Point.h>
#include <boost/foreach.hpp>
#include <fstream>
#include <iostream>
#include <glob.h>
#include <map>
#include <algorithm>

#define foreach BOOST_FOREACH

// ---------------- HEADERS ----------------
std::string getHeader(const std::string& topic) {
    if (topic == "/abdomen_force") {
        return "Fx,Fy,Fz,Fw,time";
    } else if (topic == "/tissue_force") {
        return "Fx,Fy,Fz,time";
    } else if (topic == "/alice/velocity_topic") {
        return "dX,dY,dZ,dRX,dRY,dRZ,time";
    } else if (topic == "/alice/fulcrum") {
        return "X,Y,Z,time";
    } else if (topic == "/alice/pose_topic" || topic == "/alice/effectorFinal_topic") {
        std::string header;
        for (int i = 1; i <= 16; ++i) {
            header += "V" + std::to_string(i);
            if (i != 16) header += ",";
        }
        header += ",time";
        return header;
    } else if (topic == "/alice/coordinator/goal_position") {
        return "X,Y,Z,time";
    }
    return "";
}

// ---------------- SIZE ----------------
int expectedSize(const std::string& topic) {
    if (topic == "/abdomen_force") return 4;
    if (topic == "/tissue_force") return 3;
    if (topic == "/alice/velocity_topic") return 6;
    if (topic == "/alice/fulcrum") return 3;
    if (topic == "/alice/pose_topic" || topic == "/alice/effectorFinal_topic") return 16;
    return 0;
}

// ---------------- FILE NAME ----------------
std::string sanitizeFileName(const std::string& topic) {
    std::string base = topic;
    std::replace(base.begin(), base.end(), '/', '_');
    if (!base.empty() && base[0] == '_') base = base.substr(1);
    return base + ".csv";
}

// ---------------- MAIN ----------------
int main(int argc, char** argv) {
    ros::init(argc, argv, "bag_to_csv_new_topics");

    if (argc < 2) {
        std::cerr << "Uso: " << argv[0] << " <directorio_de_bags>" << std::endl;
        return 1;
    }

    std::string bag_directory = argv[1];
    std::string pattern = bag_directory + "/*.bag";

    glob_t glob_result;
    if (glob(pattern.c_str(), GLOB_ERR, nullptr, &glob_result) != 0) {
        std::cerr << "Error al leer el directorio" << std::endl;
        return 1;
    }

    std::vector<std::string> topics = {
        "/abdomen_force",
        "/tissue_force",
        "/alice/velocity_topic",
        "/alice/pose_topic",
        "/alice/effectorFinal_topic",
        "/alice/fulcrum",
        "/alice/coordinator/goal_position"
    };

    for (size_t i = 0; i < glob_result.gl_pathc; ++i) {

        std::string bag_file = glob_result.gl_pathv[i];
        rosbag::Bag bag;

        try {
            bag.open(bag_file, rosbag::bagmode::Read);
        } catch (...) {
            std::cerr << "Error al abrir: " << bag_file << std::endl;
            continue;
        }

        rosbag::View view(bag, rosbag::TopicQuery(topics));

        std::map<std::string, std::ofstream> csv_files;
        std::map<std::string, bool> header_written;

        ros::Time start_time;
        bool first = true;

        foreach (rosbag::MessageInstance const m, view) {

            std::string topic = m.getTopic();

            // Tiempo relativo
            if (first) {
                start_time = m.getTime();
                first = false;
            }
            double rel_time = (m.getTime() - start_time).toSec();

            // Abrir archivo CSV si no existe
            if (csv_files.find(topic) == csv_files.end()) {
                std::string output_path =
                    bag_file.substr(0, bag_file.find_last_of("/\\") + 1) +
                    sanitizeFileName(topic);

                csv_files[topic].open(output_path);
                if (!csv_files[topic]) {
                    std::cerr << "No se pudo abrir CSV para: " << topic << std::endl;
                    continue;
                }
                header_written[topic] = false;
            }

            std::ofstream& file = csv_files[topic];

            // Escribir header
            if (!header_written[topic]) {
                file << getHeader(topic) << "\n";
                header_written[topic] = true;
            }

            // -------- geometry_msgs::Point (GOAL) --------
            if (topic == "/alice/coordinator/goal_position") {
                geometry_msgs::Point::ConstPtr msg =
                    m.instantiate<geometry_msgs::Point>();

                if (msg) {
                    file << msg->x << ","
                         << msg->y << ","
                         << msg->z << ","
                         << rel_time << "\n";
                }
            }
            // -------- Float64MultiArray --------
            else {
                std_msgs::Float64MultiArray::ConstPtr msg =
                    m.instantiate<std_msgs::Float64MultiArray>();

                if (!msg) continue;

                int size = expectedSize(topic);
                if (msg->data.size() < size) continue;

                for (int j = 0; j < size; ++j) {
                    file << msg->data[j];
                    if (j < size - 1) file << ",";
                }
                file << "," << rel_time << "\n";
            }
        }

        // Cerrar archivos
        for (auto& pair : csv_files) {
            pair.second.close();
            std::cout << "CSV guardado para topic: " << pair.first << std::endl;
        }

        bag.close();
    }

    globfree(&glob_result);
    return 0;
}