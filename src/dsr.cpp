#include "../include/dsr.hpp"
#include "main.h"
#include "subsystems.hpp"

//extras
int Xsen;
int Ysen;
double robot_angle;

namespace DSR{
    void add_sensor(DSRDS sensor){
        sensors.push_back(sensor);
    }

    void set_sensors(std::vector<DSRDS> sensors){
        for(unsigned int i = 0; i < sensors.size(); i++){
            add_sensor(sensors[i]);
        }
    }

    void measure_offsets(int iterations){
        double read45;
        double read30;
        double read0;
        double read0t = 0;
        double read30t = 0;
        double read45t = 0;

        //move away from wall to prevent collision
        chassis.pid_drive_set(-5_in, 40);
        chassis.pid_wait();

        //go through all the sensors and measure their offsets
        for(unsigned int i = 0; i < sensors.size(); i++){

            //reset variables after each iteration
            read0 = 0;
            read30 = 0;
            read45 = 0;
            read0t = 0;
            read30t = 0;
            read45t = 0;

            //algorithm changes a bit based on direction of the sensor
            switch(sensors[i].get_dir()){
                case Dir::Left:

                // go through multiple times and take the average
                for(int j = 0; j < iterations; j++){

                    // first turn
                    chassis.pid_turn_set(90_deg, 40);
                    chassis.pid_wait();

                    //read until actual value is read
                    for(int p = 0; p < 20; p++){
                        read0t = sensors[i].read_in();
                        if(read0t < 9999){
                            read0 = read0 + read0t;
                            break;
                        }
                    }

                    //second turn
                    chassis.pid_turn_set(60_deg, 40);
                    chassis.pid_wait();

                    //read until actual value is read
                    for(int p = 0; p < 20; p++){
                        read30t = sensors[i].read_in();
                        if(read30t < 9999){
                            read30 = read30 + read30t;
                            break;
                        }
                    }

                    //third turn
                    chassis.pid_turn_set(45_deg, 40);
                    chassis.pid_wait();

                    //read until actual value is read
                    for(int p = 0; p < 100; p++){
                        read45t = sensors[i].read_in();
                        if(read45t < 9999){
                            read45 = read45 + read45t;
                            break;
                        }
                    }
                }

                //measure offsets
                sensors[i].measure_offsets(read45 / iterations, read30 / iterations, read0 / iterations);
                break;
                //same thing for all other cases, just different angles
                case Dir::Right:
                for(int j = 0; j < iterations; j++){
                    chassis.pid_turn_set(-90_deg, 40);
                    chassis.pid_wait();
                    for(int p = 0; p < 20; p++){
                        read0t = sensors[i].read_in();
                        if(read0 < 9999){
                            read0 = read0 + read0t;
                            break;
                        }
                    }
                    chassis.pid_turn_set(-120_deg, 40);
                    chassis.pid_wait();
                    for(int p = 0; p < 20; p++){
                        read30 = read30 + sensors[i].read_in();
                        if(read30 < 9999){
                            break;
                        }
                    }
                    chassis.pid_turn_set(-135_deg, 40);
                    chassis.pid_wait();
                    for(int p = 0; p < 100; p++){
                        read45 = read45 + sensors[i].read_in();
                        if(read45 < 9999){
                            break;
                        }
                    }
                }
                sensors[i].measure_offsets(read45 / iterations, read30 / iterations, read0 / iterations);
                break;
                case Dir::Front:
                for(int j = 0; j < iterations; j++){
                    chassis.pid_turn_set(0_deg, 40);
                    chassis.pid_wait();
                    for(int p = 0; p < 100; p++){
                        read0 = read0 + sensors[i].read_in();
                        if(read0 < 9999){
                            break;
                        }
                    }
                    chassis.pid_turn_set(-30_deg, 40);
                    chassis.pid_wait();
                    for(int p = 0; p < 100; p++){
                        read30 = read30 + sensors[i].read_in();
                        if(read30 < 9999){
                            break;
                        }
                    }
                    chassis.pid_turn_set(-45_deg, 40);
                    chassis.pid_wait();
                    for(int p = 0; p < 100; p++){
                        read45 = read45 + sensors[i].read_in();
                        if(read45 < 9999){
                            break;
                        }
                    }
                }
                sensors[i].measure_offsets(read45 / iterations, read30 / iterations, read0 / iterations);
                break;
                case Dir::Back:
                for(int j = 0; j < iterations; j++){
                    chassis.pid_turn_set(180_deg, 40);
                    chassis.pid_wait();
                    for(int p = 0; p < 100; p++){
                        read0 = read0 + sensors[i].read_in();
                        if(read0 < 9999){
                            break;
                        }
                    }
                    chassis.pid_turn_set(150_deg, 40);
                    chassis.pid_wait();
                    for(int p = 0; p < 100; p++){
                        read30 = read30 + sensors[i].read_in();
                        if(read30 < 9999){
                            break;
                        }
                    }
                    chassis.pid_turn_set(135_deg, 40);
                    chassis.pid_wait();
                    for(int p = 0; p < 100; p++){
                        read45 = read45 + sensors[i].read_in();
                        if(read45 < 9999){
                            break;
                        }
                    }
                }
                sensors[i].measure_offsets(read45 / iterations, read30 / iterations, read0 / iterations);
                break;
            }
        }
    }

    void reset_tracking(Dir sensorX_dir, Dir sensorY_dir, double speed, bool slew_on, wait_type Wait_type, int sensorX_specified, int sensorY_specified){
        
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

            //turn to closest right angle
            chassis.pid_turn_set(0_deg, speed, slew_on);
            switch(Wait_type){
                case wait:
                chassis.pid_wait();
                break;
                case wait_quick:
                chassis.pid_wait_quick();
                break;
                case wait_quick_chain:
                chassis.pid_wait_quick_chain();
                break;
            }

            //thee actual alg
            if(int(sensorX_dir) == int(Left)){
                chassis.odom_x_set(sensors[Xsen].read_in());
            }else{
                chassis.odom_x_set(144 - sensors[Xsen].read_in());
            }

            if(int(sensorY_dir) == int(Back)){
                chassis.odom_y_set(sensors[Ysen].read_in());
            }else{
                chassis.odom_y_set(144 - sensors[Ysen].read_in());
            }
        }//same thing for all other cases, just different angles
        else if(45 <= robot_angle && robot_angle < 135){

            chassis.pid_turn_set(90_deg, speed, slew_on);
            switch(Wait_type){
                case wait:
                chassis.pid_wait();
                break;
                case wait_quick:
                chassis.pid_wait_quick();
                break;
                case wait_quick_chain:
                chassis.pid_wait_quick_chain();
                break;
            }

            if(int(sensorX_dir) == int(Back)){
                chassis.odom_x_set(sensors[Xsen].read_in());
            }else{
                chassis.odom_x_set(144 - sensors[Xsen].read_in());
            }

            if(int(sensorY_dir) == int(Right)){
                chassis.odom_y_set(sensors[Ysen].read_in());
            }else{
                chassis.odom_y_set(144 - sensors[Ysen].read_in());
            }
        }
        else if((135 <= robot_angle && robot_angle < 180) || (-180 <= robot_angle && robot_angle < -135)){
            
            chassis.pid_turn_set(180_deg, speed, slew_on);
            switch(Wait_type){
                case wait:
                chassis.pid_wait();
                break;
                case wait_quick:
                chassis.pid_wait_quick();
                break;
                case wait_quick_chain:
                chassis.pid_wait_quick_chain();
                break;
            }
            
            if(int(sensorX_dir) == int(Right)){
                chassis.odom_x_set(sensors[Xsen].read_in());
            }else{
                chassis.odom_x_set(144 - sensors[Xsen].read_in());
            }

            if(int(sensorY_dir) == int(Front)){
                chassis.odom_y_set(sensors[Ysen].read_in());
            }else{
                chassis.odom_y_set(144 - sensors[Ysen].read_in());
            }
        }
        else if(-135 <= robot_angle && robot_angle < -45){

            chassis.pid_turn_set(-90_deg, speed, slew_on);
            switch(Wait_type){
                case wait:
                chassis.pid_wait();
                break;
                case wait_quick:
                chassis.pid_wait_quick();
                break;
                case wait_quick_chain:
                chassis.pid_wait_quick_chain();
                break;
            }

            if(int(sensorX_dir) == int(Front)){
                chassis.odom_x_set(sensors[Xsen].read_in());
            }else{
                chassis.odom_x_set(144 - sensors[Xsen].read_in());
            }

            if(int(sensorY_dir) == int(Left)){
                chassis.odom_y_set(sensors[Ysen].read_in());
            }else{
                chassis.odom_y_set(144 - sensors[Ysen].read_in());
            }
        }
    }
}