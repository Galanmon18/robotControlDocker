#include "std_msgs/String.h"
#include <string>
#include <iostream>
#include <vector>


class PIDController {
public:
    PIDController(double kp, double ki, double kd, double time_interval, int num_controllers);
    
    std::vector<double> calculate(const std::vector<double>& error);

    std::vector<double> vel;

private:
    int num_controllers_;
    std::vector<double> kp_;
    std::vector<double> ki_;
    std::vector<double> kd_;
    double time_interval_;
    std::vector<double> previous_errors_;
    std::vector<double> integrals_;
};