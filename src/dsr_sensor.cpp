#include "../include/dsr.hpp"

//https://www.desmos.com/calculator/f8689a01ef
//this explains the numbers below, which are used to measure the offsets of the sensors from the center of the robot.
//(these numbers are converted specifically for the way i coded it so 25.4 times what the graph has)

const double Xa = 1.83195123007;
const double Xb = 4.90508341504;
const double Xc = 3.07313218497;
const double Ya = 6.83693506762;
const double Yb = 11.8419189052;
const double Yc = 4.00498383755;

DSRDS::DSRDS(int port, Dir direction, double offset) : sensor(port){
    dir = direction;
    dir_string = dir_to_string(direction);
    dir_offset = offset;
}

double DSRDS::read(){
    return sensor.get_distance();
}

double DSRDS::read_in(){
    double reading = 9999;
    for(int i = 0; reading == 9999; i++){
        reading = read();
        pros::delay(10);
    }
    return reading / 25.4;
}

void DSRDS::set_x_offset(double x){
    x_offset = x;
}

void DSRDS::set_y_offset(double y){
    y_offset = y;
}

void DSRDS::set_offsets(double x, double y){
    x_offset = x;
    y_offset = y;
}


void DSRDS::set_dir_offset(double offset){
    dir_offset = offset;
}

void DSRDS::set_dir(Dir direction){
    dir = direction;
    dir_string = dir_to_string(direction);
}

double DSRDS::get_x_offset(){
    return x_offset;
}

double DSRDS::get_y_offset(){
    return y_offset;
}

ez::pose DSRDS::get_offsets(){
    return {x_offset, y_offset};
}

Dir DSRDS::get_dir(){
    return dir;
}


double DSRDS::get_dir_offset(){
    return dir_offset;
}

std::string DSRDS::get_dir_string(){
    return dir_string;
}

void DSRDS::measure_offsets(double read45, double read30, double read0){
    switch(dir){
        case Left:
        x_offset = -Ya * read45 + Yb * read30 - Yc * read0;
        y_offset = Xa * read45 - Xb * read30 + Xc * read0;
        dir_offset = x_offset;
        break;
        case Right:
        x_offset = Ya * read45 - Yb * read30 + Yc * read0;
        y_offset = -Xa * read45 + Xb * read30 - Xc * read0;
        dir_offset = x_offset;
        break;
        case Front:
        x_offset = Xa * read45 - Xb * read30 + Xc * read0;
        y_offset = Ya * read45 - Yb * read30 + Yc * read0;
        dir_offset = y_offset;
        break;
        case Back:
        x_offset = -Xa * read45 + Xb * read30 - Xc * read0;
        y_offset = -Ya * read45 + Yb * read30 - Yc * read0;
        dir_offset = y_offset;
        break;
    }
}