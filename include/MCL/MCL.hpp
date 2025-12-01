#pragma once
#include <vector>
#include "EZ-Template/util.hpp"
#include "MCL-sensor.hpp"

struct MCLpose{
    double x;
    double y;
    double theta;
};

class MCL{
    public:
    std::vector<MCLDS> sensors;
    void add_sensor(MCLDS& sensor);
    pose get_pose();
};