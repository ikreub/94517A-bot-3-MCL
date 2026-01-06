#pragma once

#include <string>
#include <vector>
#include "../include/EZ-Template/api.hpp"
#include "pros/distance.hpp"
#include "main.h"

//wait wrappers for odom reset customization
enum wait_type{
    wait,
    wait_quick,
    wait_quick_chain
};

//sensor direction enum
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

//sensors!!!
class DSRDS{
    public:

    //constructor
    DSRDS(int port, Dir direction, double x_offset, double y_offset);

    //read sensor distance in inches
    double read_in();

    //set x offset
    void set_x_offset(double x);

    //set y offset
    void set_y_offset(double y);

    //set ooffsets
    void set_offsets(double x, double y);

    //set direction
    void set_dir(Dir direction);

    //get x offset
    double get_x_offset();

    //get_y_offset
    double get_y_offset();

    //get offsets
    ez::pose get_offsets();

    //get direction
    Dir get_dir();

    //get direction as text for debugging
    std::string get_dir_string();

    //measure sensor offsets
    void measure_offsets(double read45, double read30, double read0);
    private:

    //offsets
    ez::pose offsets = {0,0};

    //direction
    Dir dir;

    //direction in text
    std::string dir_string;

    //actual sensor
    pros::Distance sensor;

    //helper function
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

//DSR functions
namespace DSR{

    //set dsr sensors
    void set_sensors(std::vector<DSRDS> sensors);

    //add one sensor to the list of dsr sensors
    void add_sensor(DSRDS sensor);

    //the actual dsr algorithm
    void reset_tracking(Dir sensorX_dir, Dir sensorY_dir, double speed, bool slew_on = chassis.slew_turn_get(), wait_type Wait_type = wait, int sensorX_specified = 1, int sensorY_specified = 1 );
    
    //measure all the distance sensor ofsets
    void measure_offsets(int iterations);

    //sensors
    inline std::vector<DSRDS> sensors = {};
}