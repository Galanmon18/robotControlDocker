#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Float64MultiArray.h>
#include <boost/foreach.hpp>
#include <fstream>
#include <iostream>
#include <glob.h> // Para buscar archivos .bag
#include <string>

#define foreach BOOST_FOREACH

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bag_to_csv");

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

    for (size_t i = 0; i < glob_result.gl_pathc; ++i) {
        std::string bag_file = glob_result.gl_pathv[i];
        std::string output_file = bag_file;
        output_file.replace(output_file.end() - 4, output_file.end(), ".csv");

        rosbag::Bag bag;
        try {
            bag.open(bag_file, rosbag::bagmode::Read);
        } catch (rosbag::BagIOException &e) {
            std::cerr << "Error al abrir el archivo: " << bag_file << std::endl;
            continue;
        }

        std::vector<std::string> topics = {"/alice/polar/current", "/alice/polar/des", "/alice/polar/delta", "/alice/velocity_topic", "/alice/pose_topic", "/alice/effectorFinal_topic", "/alice/fulcrum"};
        rosbag::View view(bag, rosbag::TopicQuery(topics));

        std::ofstream csv_file(output_file);
        bool header_written = false;

        foreach(rosbag::MessageInstance const m, view) {
            std_msgs::Float64MultiArray::ConstPtr msg = m.instantiate<std_msgs::Float64MultiArray>();
            if (msg != nullptr && msg->data.size() >= 3) {
                std::string topic_name = m.getTopic();

                // Escribir la cabecera según el topic (una sola vez)
                if (!header_written) {
                    if (topic_name.find("/alice/polar/") == 0) {
                        csv_file << "# Alpha,Beta,Rho";
                    } else if (topic_name == "/alice/velocity_topic") {
                        csv_file << "# dX,dY,dZ,dRX,dRY,dRZ";
                    } else if (topic_name == "/alice/fulcrum") {
                        csv_file << "# X,Y,Z";
                    } else if (topic_name == "/alice/pose_topic" || topic_name == "/alice/effectorFinal_topic") {
                        csv_file << "# V1,V2,V3,V4,V5,V6,V7,V8,V9,V10,V11,V12,V13,V14,V15,V16";
                    }
                    csv_file << std::endl;
                    header_written = true;
                }

                if (topic_name.find("/alice/polar/") == 0 && msg->data.size() >= 3) {
                    csv_file << msg->data[0] << "," << msg->data[1] << "," << msg->data[2] << std::endl;
                } else if (topic_name == "/alice/velocity_topic" && msg->data.size() >= 6) {
                    csv_file << msg->data[0] << "," << msg->data[1] << "," << msg->data[2] << ","
                             << msg->data[3] << "," << msg->data[4] << "," << msg->data[5] << std::endl;
                } else if (topic_name == "/alice/fulcrum" && msg->data.size() >= 3) {
                    csv_file << msg->data[0] << "," << msg->data[1] << "," << msg->data[2] << std::endl;
                } else if ((topic_name == "/alice/pose_topic" || topic_name == "/alice/effectorFinal_topic") && msg->data.size() >= 16) {
                    for (size_t j = 0; j < 16; ++j) {
                        csv_file << msg->data[j];
                        if (j < 15) csv_file << ",";
                    }
                    csv_file << std::endl;
                }
            }
        }

        csv_file.close();
        bag.close();
        std::cout << "Datos guardados en " << output_file << std::endl;
    }

    globfree(&glob_result);
    return 0;
}
