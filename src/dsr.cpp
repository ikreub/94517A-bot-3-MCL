#include "../include/dsr.hpp"
#include <cmath>
#include "main.h"
#include "subsystems.hpp"

//extras
int Xsen;
int Ysen;
double robot_angle;

void read_sensor_measure(double& read, DSRDS& sensor){
    double readt;

    //make sure the sensor is reading something valid, and if it is, add it to the total
    for(int p = 0; p < 20; p++){
        readt = sensor.read_in();
        if(readt < 9999){
            read = read + readt;
            break;
        }
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
    
    //reset the tracking values based on the sensor readings and the direction of the sensors
    if(int(senX_dir) == int(Xdir)){
        chassis.odom_x_set(DSR::sensors[Xsen].read_in() - DSR::sensors[Xsen].get_dir_offset());
    }else{
        chassis.odom_x_set(144 - DSR::sensors[Xsen].read_in() + DSR::sensors[Xsen].get_dir_offset());
    }

    if(int(senY_dir) == int(Ydir)){
        chassis.odom_y_set(DSR::sensors[Ysen].read_in() - DSR::sensors[Ysen].get_dir_offset());
    }else{
        chassis.odom_y_set(144 - DSR::sensors[Ysen].read_in() + DSR::sensors[Ysen].get_dir_offset());
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
        robot_angle = fmod(chassis.odom_theta_get() - 180, 360) + 180;

        //find direction
        if(-45 <= robot_angle && robot_angle < 45){

            //the actual alg
            odom_reset(sensorX_dir, Left, Xsen, sensorY_dir, Back, Ysen);
        }//same thing for all other cases, just different angles
        else if(45 <= robot_angle && robot_angle < 135){

            odom_reset(sensorX_dir, Back, Xsen, sensorY_dir, Right, Ysen);
        }
        else if((135 <= robot_angle && robot_angle < 180) || (-180 <= robot_angle && robot_angle < -135)){
            
            odom_reset(sensorX_dir, Right, Xsen, sensorY_dir, Front, Ysen);
        }
        else if(-135 <= robot_angle && robot_angle < -45){

            odom_reset(sensorX_dir, Front, Xsen, sensorY_dir, Left, Ysen);
        }
    }
}