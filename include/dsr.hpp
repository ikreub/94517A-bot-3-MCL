#pragma once

#include <string>
#include <vector>
#include "../include/EZ-Template/api.hpp"
#include "pros/distance.hpp"
#include "main.h"

enum wait_type{
    wait,
    wait_quick,
    wait_quick_chain
};

enum Dir{
    l = 1,
    r = 2,
    f = 3,
    b = 4,
    left = l,
    right = r,
    front = f,
    back = b,
    L = l,
    R = r,
    F = f,
    B = b,
    Left = l,
    Right = r,
    Front = f,
    Back = b,
    LEFT = l,
    RIGHT = r,
    FRONT = f,
    BACK = b
};

class DSRDS{
    public:
    DSRDS(int port, Dir direction, double x_offset, double y_offset);
    double read_in();
    void set_x_offset(double x);
    void set_y_offset(double y);
    void set_offsets(double x, double y);
    void set_dir(Dir direction);
    double get_x_offset();
    double get_y_offset();
    ez::pose get_offsets();
    Dir get_dir();
    std::string get_dir_string();
    void measure_offsets(double read45, double read30, double read0);
    private:
    ez::pose offsets = {0,0};
    Dir dir;
    std::string dir_string;
    pros::Distance sensor;
    std::string dir_to_string(Dir direction){
        switch(int(direction)){
            case int(l):
                return "left";
            case int(r):
                return "right";
            case int(f):
                return "front";
            case int(b):
                return "back";
            default:
                return "";
        }
    }
};

namespace DSR{
    void set_sensors(std::vector<DSRDS> sensors);
    void add_sensor(DSRDS sensor);
    void reset_tracking(Dir sensorX_dir, Dir sensorY_dir, double speed, bool slew_on = chassis.slew_turn_get(), wait_type Wait_type = wait, int sensorX_specified = 1, int sensorY_specified = 1 );
    void measure_offsets(int iterations);
    inline std::vector<DSRDS> sensors = {};
}