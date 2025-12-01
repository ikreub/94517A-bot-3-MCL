#pragma once
#include <vector>
#include "EZ-Template/util.hpp"
#include "MCL-sensor.hpp"
#include "MCL-particals.hpp"

struct MCLpose{
    double x;
    double y;
    double theta;
};

class MCL{
    public:
    MCL();
    void add_sensor(MCLDS sensor);
    void set_sensors(std::vector<MCLDS>& sensors);
    pose get_pose();
    void MCL_task();
    double get_certainty();
    private:
    double Certainty;
    std::vector<partical> particals;
    std::vector<pose> get_particals(double scaler, double theta);
    int Density = 13;
    std::vector<MCLDS> sensors;
    std::vector<pose> partical_spread;
    std::vector<double> get_distances();
};
