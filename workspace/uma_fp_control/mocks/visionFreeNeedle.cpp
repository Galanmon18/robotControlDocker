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
    ros::init(argc, argv, "visionMoch");
    ros::NodeHandle n;
    //Para realizar la lectura por pantalla
    system("stty raw"); //evita presionar enter tras cada letra
    bool salir=false; //sale tras pulsar x
    ros::Publisher pub1 = n.advertise<std_msgs::Bool>("vision_topic", 1000);
    //ros data to publish a boolean true or false
    std_msgs::Bool freeStitch;
    freeStitch.data = true;
    while (ros::ok() && !salir){
        char input=getchar();
        switch (input){
            case 'f': 
                freeStitch.data = true;
                ROS_INFO("\r\n free \r\n");
                break;  
            case 'g':
                freeStitch.data = false;
                ROS_INFO("\r\n no free \r\n");
                break;  
            case 'x':
                salir = true;
                system("salir");
                break;         
        }
        pub1.publish(freeStitch);
    }
  return 0;
}