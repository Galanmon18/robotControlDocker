#include <dependecies/computePID.hpp>

PIDController::PIDController(double kp, double ki, double kd, double time_interval, int num_controllers)
: num_controllers_(num_controllers),
kp_(std::vector<double>(num_controllers, kp)),
ki_(std::vector<double>(num_controllers, ki)),
kd_(std::vector<double>(num_controllers, kd)),
time_interval_(time_interval),
previous_errors_(num_controllers, 0),
integrals_(num_controllers, 0) {
    vel.resize(num_controllers);
    std::cout << "kp= " << kp << std::endl;
    std::cout << "ki= " << ki << std::endl;
    std::cout << "kd= " << kd << std::endl;
    std::cout << "time_interval= " << time_interval << std::endl;
    std::cout << "num_controllers= " << num_controllers << std::endl;
}

std::vector<double> PIDController::calculate(const std::vector<double>& error) {
    for (int i = 0; i < num_controllers_; ++i) {
        integrals_[i] += error[i] * time_interval_;
        double derivative = (error[i] - previous_errors_[i]) / time_interval_;
        double output = kp_[i] * error[i] + ki_[i] * integrals_[i] + kd_[i] * derivative;
        previous_errors_[i] = error[i];
        vel[i] = output;
    }
    return vel;
}