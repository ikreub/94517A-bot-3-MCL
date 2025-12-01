#pragma once
#include <vector>
#include <cmath>
#include "EZ-Template/util.hpp"
#include "../include/MCL/MCL-sensor.hpp"

class partical{
    public:
    partical(double x, double y, double theta, std::vector<MCLDS>& sensores);
    std::vector<MCLDS> sensors;
    double x;
    double y;
    double theta;
    double weight;
    void displace(double dtheta, double dx, double dy);
    void get_weight(std::vector<double> sensor_vals, double allowed_deviation);
    private:
    std::vector<double> scale(double scaler);
    std::vector<double> get_sim_distance();
};