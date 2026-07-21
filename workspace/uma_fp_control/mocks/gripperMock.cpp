// lee por teclados comandos de teleoperacion
// el resultado se mando por el topic cmd_key
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <stdlib.h>
#include <string.h>
#include "std_msgs/Bool.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gripperMoch");
    ros::NodeHandle n;
    //Para realizar la lectura por pantalla
    system("stty raw"); //evita presionar enter tras cada letra
    bool salir=false; //sale tras pulsar x
    ros::Publisher pub1 = n.advertise<std_msgs::Bool>("gripperR_topic", 1000);
    ros::Publisher pub2 = n.advertise<std_msgs::Bool>("gripperL_topic", 1000);
    //ros data to publish a boolean true or false
    std_msgs::Bool closeRgripper;
    std_msgs::Bool closeLgripper;
    closeRgripper.data = false;
    closeLgripper.data = false;
    while (ros::ok() && !salir){
        char input=getchar();
        switch (input){
            case 'o': 
                closeRgripper.data = false;
                ROS_INFO("\r\n Open R gripper \r\n");
                break;  
            case 'p':
                closeRgripper.data = true;
                ROS_INFO("\r\n Close R gripper \r\n");
                break; 
            case 'w': 
                closeLgripper.data = false;
                ROS_INFO("\r\n Open L gripper \r\n");
                break;    
            case 'q':
                closeLgripper.data = true;
                ROS_INFO("\r\n Close L gripper \r\n");
                break;   
            case 'x':
                salir = true;
                system("salir");
                break;         
        }
        pub1.publish(closeRgripper);
        pub2.publish(closeLgripper);
    }
  return 0;
}