#include <iostream>
#include <vector>
#include "dependecies/rho_estimation.hpp"
#include <cmath>
#include "ros/ros.h"

int main()
{
    rhoEstimation sensors(
        "192.168.1.1",                 // FT IP (no usada)
        "/dev/ttyACM0",     // Arduino
        true,              // FT desactivado
        true                // Distancia activada
    );

    std::vector<double> forces;
    double distance, threshold, dCalc,toolLength;
    double Frcm_x, Frcm_y, Frcm_z, Fip_x, Fip_y, Fip_z;

    threshold = 0.01;
    toolLength=20;

    //reset sensor
    sensors.tareFT();

    while (true) {
        if (sensors.readFT(forces)){
            double forceNorm =
                std::sqrt(forces[0] * forces[0] +
                        forces[1] * forces[1]);

            double torqueNorm =
                std::sqrt(forces[3] * forces[3] +
                        forces[4] * forces[4]);
            dCalc = torqueNorm/forceNorm;
        } else {
            dCalc = 0;
        }

        std::cout
            << "dCalc="
            << dCalc*1000
            << " mm"
            << std::endl;

        if (sensors.readDistanceBlocking(distance, 500)){
            std::cout
                << "Distance="
                << distance
                << " mm"
                << std::endl;
        }
        else{
            std::cout << "Distance timeout / no data" << std::endl;
        }
        if (abs(dCalc-distance) < threshold){
            Frcm_x= (forces[4]-forces[0]*toolLength)/(dCalc-distance);
            Frcm_y= Frcm_x= (-forces[3]-forces[1]*toolLength)/(dCalc-distance);
            Fip_x= forces[0] - Frcm_x;
            Fip_y= forces[1] - Frcm_y;

            Frcm_z= 0;
            Fip_z= forces[2];
        }else {
            Frcm_x= forces[0];
            Frcm_y= forces[1];
            Frcm_z= 0;
            Fip_x= 0;
            Fip_y= 0;

            Frcm_z= forces[2];
            Fip_z= 0;
        }
        std::cout
                << "Frcm_x=" << Frcm_x
                << " Frcm_y=" << Frcm_y
                << " Frcm_z=" << Frcm_z
                << " Fip_x=" << Fip_x
                << " Fip_y=" << Fip_y
                << " Fip_z=" << Fip_z
                << std::endl;
        std::cout << "==========================================" << std::endl;
    }

    return 0;
}