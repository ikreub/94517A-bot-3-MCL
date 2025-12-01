#pragma once
#include <vector>
#include "EZ-Template/util.hpp"
#include "../include/MCL/MCL-sensor.hpp"

class partical{
    public:
    partical(double x, double y, double theta, std::vector<MCLDS> sensors);
    std::vector<MCLDS> sensors;
    double x;
    double y;
    double theta;
    double weight;
    void displace(double dtheta, double dx, double dy);
    void get_weight(double sensor_val, double expected_val, double deviation);
    private:
    double get_sim_distance();
};