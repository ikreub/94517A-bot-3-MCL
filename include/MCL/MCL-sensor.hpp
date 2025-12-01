#pragma once
#include "pros/distance.hpp"
#include "subsystems.hpp"
#include <vector>

class MCLDS{
    public:
    pros::Distance dist_sen;
    MCLDS(Dir direction, int port, double offset_lr, double offset_fb);
    void set_offset_x(double offset);
    void set_offset_y(double offset);
    void set_offset_xy(double offset_x, double offset_y);
    void set_dir(Dir direction);
    double get_offset_x();
    double get_offset_y();
    std::vector<double> get_offset_xy();
    double read();
    Dir get_dir();
    private:
    double offset_x;
    double offset_y;
    Dir dir;
};