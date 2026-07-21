#ifndef SENSOR_MANAGER_HPP
#define SENSOR_MANAGER_HPP
#include <vector>
#include <string>
#include <memory>
#include <stdexcept>
#include <chrono>
#include <unistd.h>
#include <iostream>
#include "dependecies/hex_ft_udp.hpp"
#include "dependecies/DistanceSensorSerial.hpp"
class rhoEstimation
{
private:
    std::unique_ptr<FTSensor> ftSensor;
    std::unique_ptr<DistanceSensorSerial> distanceSensor;
public:
    rhoEstimation(const std::string& ft_ip,
                  const std::string& serial_port,
                  bool useFT = true,
                  bool useDistance = true)
    {
        if (useFT)
        {
            try
            {
                ftSensor = std::make_unique<FTSensor>(ft_ip);
            }
            catch (const std::exception& e)
            {
                ftSensor.reset();
            }
        }
        if (useDistance)
        {
            try
            {
                distanceSensor = std::make_unique<DistanceSensorSerial>(serial_port);
            }
            catch (const std::exception& e)
            {
                distanceSensor.reset();
            }
        }
    }
    bool readFT(std::vector<double>& forces)
    {
        if (!ftSensor)
            return false;
        return ftSensor->readFT(forces);
    }
    bool readDistance(double& distance)
    {
        if (!distanceSensor)
            return false;
        return distanceSensor->readDistance(distance);
    }
    bool readDistanceBlocking(double& distance, int timeoutMs = 500)
    {
        if (!distanceSensor)
            return false;
        auto start = std::chrono::steady_clock::now();
        while (true)
        {
            if (distanceSensor->readDistance(distance))
                return true;
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > timeoutMs)
                return false;
            usleep(5000);
        }
    }
    bool tareFT()
    {
        if (!ftSensor) return false;
        return ftSensor->tareSensor();
    }
};
#endif