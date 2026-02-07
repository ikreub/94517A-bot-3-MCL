#include "../include/dsr.hpp"
#include <stdlib.h>
#include <cmath>
#include "EZ-Template/util.hpp"
#include "main.h"
#include "subsystems.hpp"

//22.83
//46.38
//69.97
//93.7
//117.32

//extras
int Xsen;
int Ysen;
double robot_angle;
const double feild_size = 144;//140.94488189; //currently code is designed for 144 inches feild

const bool debug = false;

double deg_mod(double num){
    bool is_done = false;
    while(!is_done){
        if(num > 180){
            num -= 360;
        }else if(num < -180){
            num += 360;
        }else{
            is_done = true;
        }
    }
    return num;
}

void read_sensor_measure(double& read, DSRDS& sensor){
    double readt = 9999;

    //make sure the sensor is reading something valid, and if it is, add it to the total
    while(readt >= 9999 || readt < 0){
        readt = sensor.read_in();
        if(readt < 9999 / 25.4){
            read = readt;
            break;
        }
    }
}

double dir_trig(Dir direction, double num){
    switch(direction){
        case f:
        return abs(num * cos(util::to_rad(chassis.odom_theta_get())));
        break;
        case r:
        return abs(num * sin(util::to_rad(chassis.odom_theta_get())));
        break;
        case l:
        return abs(num * sin(util::to_rad(chassis.odom_theta_get())));
        break;
        case b:
        return abs(num * cos(util::to_rad(chassis.odom_theta_get())));
        break;
    }
}

void triple_read(int angle, int iterations, int i){
    double read45 = 0;
    double read30 = 0;
    double read0 = 0;

    //go through multiple times and take the average
    for(int j = 0; j < iterations; j++){

        chassis.pid_turn_set(angle, 40);
        chassis.pid_wait();
        read_sensor_measure(read0, DSR::sensors[i]);

        chassis.pid_turn_set(angle - 30, 40);
        chassis.pid_wait();
        read_sensor_measure(read30, DSR::sensors[i]);

        chassis.pid_turn_set(angle - 45, 40);
        chassis.pid_wait();
        read_sensor_measure(read45, DSR::sensors[i]);
    }

    DSR::sensors[i].measure_offsets(read45 / iterations, read30 / iterations, read0 / iterations);
}

void odom_reset(Dir senX_dir, Dir Xdir, int Xsen, Dir senY_dir, Dir Ydir, int Ysen){
    double read;
    //reset the tracking values based on the sensor readings and the direction of the sensors
    read = dir_trig(Ydir,DSR::sensors[Xsen].read_in() + DSR::sensors[Xsen].get_dir_offset());
    if(read < 9999 / 25.4){
        if(int(senX_dir) == int(Xdir)){
            chassis.odom_x_set(read);
            if(debug){
                ez::screen_print("X: true read: " + util::to_string_with_precision(read) + " Raw: " + util::to_string_with_precision(DSR::sensors[Xsen].read_in() + DSR::sensors[Xsen].get_dir_offset()),5);
            }
        }else{
            chassis.odom_x_set(feild_size - read);
            if(debug){
                ez::screen_print("X: false read: " + util::to_string_with_precision(read) + " Raw: " + util::to_string_with_precision(DSR::sensors[Xsen].read_in() + DSR::sensors[Xsen].get_dir_offset()),5);
            }
        }
    }else{
        ez::screen_print("error: no legnth read", 5);
    }

    read = dir_trig(Ydir,DSR::sensors[Ysen].read_in() + DSR::sensors[Ysen].get_dir_offset());
    if(read < 9999 / 25.4){
        if(int(senY_dir) == int(Ydir)){
            chassis.odom_y_set(read);
            if(debug){
                ez::screen_print("Y: true read: " + util::to_string_with_precision(read) + " Raw: " + util::to_string_with_precision(DSR::sensors[Ysen].read_in() + DSR::sensors[Ysen].get_dir_offset()), 6);
            }
        }else{
            chassis.odom_y_set(feild_size - read);
            if(debug){
                ez::screen_print("Y: false read: " + util::to_string_with_precision(read) + " Raw: " + util::to_string_with_precision(DSR::sensors[Ysen].read_in() + DSR::sensors[Ysen].get_dir_offset()), 6);
            }
        }
    }else{
        ez:screen_print("error: no legnth read", 6);
    }
}

namespace DSR{
    void add_sensor(DSRDS& sensor){
        sensors.push_back(sensor);
    }

    void set_sensors(std::vector<DSRDS> sensors){
        for(unsigned int i = 0; i < sensors.size(); i++){
            add_sensor(sensors[i]);
        }
    }

    void measure_offsets(int iterations){

        //move away from wall to prevent collision
        chassis.pid_drive_set(-5_in, 40);
        chassis.pid_wait();

        //go through all the sensors and measure their offsets
        for(unsigned int i = 0; i < sensors.size(); i++){

            //algorithm changes angle based on direction of the sensor
            switch(sensors[i].get_dir()){
                case Left:
                triple_read(90, iterations, i);
                break;
                case Right:
                triple_read(-90, iterations, i);
                break;
                case Front:
                triple_read(0, iterations, i);
                break;
                case Back:
                triple_read(180, iterations, i);
                break;
            }
        }
    }

    void reset_tracking(Dir sensorX_dir, Dir sensorY_dir, int sensorX_specified, int sensorY_specified){
        
        if(debug){
            ez::screen_print(util::to_string_with_precision(chassis.odom_theta_get()), 3);
        }
        //figure out which sensors are being used for x and y semi efficiently by counting down the number of specified sensors until we find the one we want
        for(unsigned int i = 0; i < sensors.size(); i++){
            if(sensors[i].get_dir() == sensorX_dir){
                sensorX_specified--;
                if(sensorX_specified == 0){
                    Xsen = i;
                    break;
                }
            }
        }
        for(unsigned int i = 0; i < sensors.size(); i++){
            if(sensors[i].get_dir() == sensorY_dir){
                sensorY_specified--;
                if(sensorY_specified == 0){
                    Ysen = i;
                    break;
                }
            }
        }

        //get the robot angle to determine which way the bot is facing
        robot_angle = deg_mod(chassis.odom_theta_get());

        //find direction
        if(-45 <= robot_angle && robot_angle < 45){

            if(debug){
                ez::screen_print("Front", 4);
            }
            //the actual alg
            odom_reset(sensorX_dir, Left, Xsen, sensorY_dir, Back, Ysen);
        }//same thing for all other cases, just different angles
        else if(45 <= robot_angle && robot_angle < 135){

            if(debug){
                ez::screen_print("Right", 4);
            }
            odom_reset(sensorX_dir, Back, Xsen, sensorY_dir, Right, Ysen);
        }
        else if((135 <= robot_angle && robot_angle < 180) || (-180 <= robot_angle && robot_angle < -135)){
            
            if(debug){
                ez::screen_print("Back", 4);
            }
            odom_reset(sensorX_dir, Right, Xsen, sensorY_dir, Front, Ysen);
        }
        else if(-135 <= robot_angle && robot_angle < -45){
            if(debug){
                ez::screen_print("Left", 4);
            }
            odom_reset(sensorX_dir, Front, Xsen, sensorY_dir, Left, Ysen);
        }else{
            ez::screen_print("error:angle out of bounds", 4);
        }
        if(debug){
            ez::screen_print("pose: (" + util::to_string_with_precision(chassis.odom_x_get()) + ", " + util::to_string_with_precision(chassis.odom_y_get()) + ")", 7);
            while(!master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
                pros::delay(10);
            }
        }
    }
}