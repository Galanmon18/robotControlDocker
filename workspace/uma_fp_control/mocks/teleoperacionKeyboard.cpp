// lee por teclados comandos de teleoperacion
// el resultado se mando por el topic cmd_key
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <stdlib.h>
#include <string.h>
#include "geometry_msgs/Point.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard");
    ros::NodeHandle n;
    //Para realizar la lectura por pantalla
    system("stty raw"); //evita presionar enter tras cada letra
    bool salir=false; //sale tras pulsar x
    ros::Publisher pub = n.advertise<geometry_msgs::Point>("haptic_topic", 1000);
    while (ros::ok() && !salir){
        char input=getchar();
        geometry_msgs::Point msg;
        msg.x = 0;
        msg.y = 0;
        msg.z = 0;
        switch (input){
            case '1': 
                msg.x = 0.023;
                msg.y = 0;
                msg.z = 0;
                ROS_INFO("\r\n Mover adelante en X \r\n");
                break;  
            case '2':
                msg.x = 0;
                msg.y = 0.023;
                msg.z = 0;
                ROS_INFO("\r\n Mover adelante en Y \r\n");
                break; 
            case '3': 
                msg.x = 0;
                msg.y = 0;
                msg.z = 0.01;
                ROS_INFO("\r\n Mover adelante en Z \r\n");
                break;    
            case '4':
                msg.x = -0.02;
                msg.y = 0;
                msg.z = 0;
                ROS_INFO("\r\n Mover atras en X \r\n");
                break;   
            case '5':
                msg.x = 0;
                msg.y = -0.023;
                msg.z = 0;
                ROS_INFO("\r\n Mover atras en Y \r\n");
                break;
            case '6': 
                msg.x = 0;
                msg.y = 0;
                msg.z = -0.03;
                ROS_INFO("\r\n Mover atras en Z \r\n");
                break; 
            case '7':
                msg.x = -0.015;
                msg.y = -0.0;
                msg.z = -0.038;
                ROS_INFO("\r\n Diagonal X Y \r\n");
                break;  
            case '8':
                msg.x = +0.015;
                msg.y = +0.0;
                msg.z = +0.038;
                ROS_INFO("\r\n Diagonal X -Y \r\n");
                break;
            case '9':
                msg.x = +0.0;
                msg.y = -0.0;
                msg.z = 0;
                ROS_INFO("\r\n Diagonal -X Y . Pulsa x para salir\r\n");
                break;
            case 'x':
                salir = true;
                system("stty cooked");
                break;         
        }
        pub.publish(msg); //escucha por si alguien quiere hablar con mi nodo 
    }
  return 0;
}