#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point.h>
#include <uma_fp_control/stitchArea.h>
#include <boost/foreach.hpp>
#include <fstream>
#include <iostream>
#include <glob.h>
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

        std::vector<std::string> topics = {
            "/alice/velocity_topic", "/alice/pose_topic", "/alice/effectorFinal_topic", "/gripper_r", "/gripper_l",
            "/bob/velocity_topic", "/bob/pose_topic", "/bob/effectorFinal_topic",
            "/phase", "/current_area", "/availableIns", "/availableExt", "/finish",
            "force_r", "force_l"
        };
        rosbag::View view(bag, rosbag::TopicQuery(topics));

        std::ofstream csv_file(output_file);
        if (!csv_file.is_open()) {
            std::cerr << "Error al abrir el archivo CSV: " << output_file << std::endl;
            bag.close();
            continue;
        }

        // Write header for each topic type
        bool header_written = false;
        std::string last_topic = "";

        foreach(rosbag::MessageInstance const m, view) {
            std::string topic_name = m.getTopic();

            // If topic changes, reset header_written to allow new header
            if (topic_name != last_topic) {
                header_written = false;
                last_topic = topic_name;
            }

            // Handle Float64MultiArray messages
            std_msgs::Float64MultiArray::ConstPtr float_msg = m.instantiate<std_msgs::Float64MultiArray>();
            if (float_msg != nullptr) {
                if (!header_written) {
                    if (topic_name == "/alice/velocity_topic" || topic_name == "/bob/velocity_topic") {
                        csv_file << "# dX,dY,dZ,dRX,dRY,dRZ" << std::endl;
                    } else if (topic_name == "/alice/pose_topic" || topic_name == "/alice/effectorFinal_topic" ||
                               topic_name == "/bob/pose_topic" || topic_name == "/bob/effectorFinal_topic") {
                        csv_file << "# V1,V2,V3,V4,V5,V6,V7,V8,V9,V10,V11,V12,V13,V14,V15,V16" << std::endl;
                    }
                    header_written = true;
                }

                if ((topic_name == "/alice/velocity_topic" || topic_name == "/bob/velocity_topic") && float_msg->data.size() >= 6) {
                    csv_file << float_msg->data[0] << "," << float_msg->data[1] << "," << float_msg->data[2] << ","
                             << float_msg->data[3] << "," << float_msg->data[4] << "," << float_msg->data[5] << std::endl;
                } else if ((topic_name == "/alice/pose_topic" || topic_name == "/alice/effectorFinal_topic" ||
                            topic_name == "/bob/pose_topic" || topic_name == "/bob/effectorFinal_topic") && float_msg->data.size() >= 16) {
                    for (size_t j = 0; j < 16; ++j) {
                        csv_file << float_msg->data[j];
                        if (j < 15) csv_file << ",";
                    }
                    csv_file << std::endl;
                }
            }

            // Handle UInt8 messages (gripper_r, gripper_l)
            std_msgs::UInt8::ConstPtr uint8_gripper_msg = m.instantiate<std_msgs::UInt8>();
            if (uint8_gripper_msg != nullptr && (topic_name == "/gripper_r" || topic_name == "/gripper_l")) {
                if (!header_written) {
                    csv_file << "# GripperState" << std::endl;
                    header_written = true;
                }
                csv_file << (int)uint8_gripper_msg->data << std::endl;
            }
            std_msgs::UInt8::ConstPtr uint8_force_msg = m.instantiate<std_msgs::UInt8>();
            if (uint8_force_msg != nullptr && (topic_name == "/force_r" || topic_name == "/force_l")) {
                if (!header_written) {
                    csv_file << "# GripperForce" << std::endl;
                    header_written = true;
                }
                csv_file << (int)uint8_force_msg->data << std::endl;
            }

            // Handle UInt8MultiArray messages (finish, availableIns, availableExt)
            std_msgs::UInt8MultiArray::ConstPtr uint8_msg = m.instantiate<std_msgs::UInt8MultiArray>();
            if (uint8_msg != nullptr && (topic_name == "/finish" || topic_name == "/availableIns" || topic_name == "/availableExt")) {
                if (!header_written) {
                    csv_file << "# Values" << std::endl;
                    header_written = true;
                }
                for (size_t j = 0; j < uint8_msg->data.size(); ++j) {
                    csv_file << (int)uint8_msg->data[j];
                    if (j < uint8_msg->data.size() - 1) csv_file << ",";
                }
                csv_file << std::endl;
            }

            // Handle Int32 messages (phase)
            std_msgs::Int32::ConstPtr int32_msg = m.instantiate<std_msgs::Int32>();
            if (int32_msg != nullptr && topic_name == "/phase") {
                if (!header_written) {
                    csv_file << "# PhaseValue" << std::endl;
                    header_written = true;
                }
                csv_file << int32_msg->data << std::endl;
            }

            // Handle uma_fp_control/stitchArea messages for current_area
            uma_fp_control::stitchArea::ConstPtr stitch_msg = m.instantiate<uma_fp_control::stitchArea>();
            if (stitch_msg != nullptr && topic_name == "/current_area") {
                if (!header_written) {
                    csv_file << "# InsertionX,InsertionY,InsertionZ,ExtractionX,ExtractionY,ExtractionZ,ChangeX,ChangeY,ChangeZ" << std::endl;
                    header_written = true;
                }
                std::cout << "Processing /current_area message: "
                          << "Insertion(" << stitch_msg->insertion.x << "," << stitch_msg->insertion.y << "," << stitch_msg->insertion.z << "), "
                          << "Extraction(" << stitch_msg->extraction.x << "," << stitch_msg->extraction.y << "," << stitch_msg->extraction.z << "), "
                          << "Change(" << stitch_msg->change.x << "," << stitch_msg->change.y << "," << stitch_msg->change.z << ")" << std::endl;
                csv_file << stitch_msg->insertion.x << "," << stitch_msg->insertion.y << "," << stitch_msg->insertion.z << ","
                         << stitch_msg->extraction.x << "," << stitch_msg->extraction.y << "," << stitch_msg->extraction.z << ","
                         << stitch_msg->change.x << "," << stitch_msg->change.y << "," << stitch_msg->change.z << std::endl;
            }
        }

        csv_file.close();
        bag.close();
        std::cout << "Datos guardados en " << output_file << std::endl;
    }

    globfree(&glob_result);
    return 0;
}