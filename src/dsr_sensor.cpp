#include "../include/dsr.hpp"

//https://www.desmos.com/calculator/f8689a01ef
//this explains the numbers below, which are used to measure the offsets of the sensors from the center of the robot.

const double Xa = 0.0721240641761;
const double Xb = 0.193113520277;
const double Xc = 0.120989456101;
const double Ya = 0.269170671954;
const double Yb = 0.466217279731;
const double Yc = 0.157676529037;

DSRDS::DSRDS(int port, Dir direction, double x_offset, double y_offset) : sensor(port){
    this->dir = direction;
    this->dir_string = dir_to_string(direction);
    this->offsets = {x_offset, y_offset};
}

double DSRDS::read_in(){
    return sensor.get_distance() / 25.4;
}

void DSRDS::set_x_offset(double x){
    this->offsets.x = x;
}

void DSRDS::set_y_offset(double y){
    this->offsets.y = y;
}

void DSRDS::set_offsets(double x, double y){
    this->offsets = {x, y};
}

void DSRDS::set_dir(Dir direction){
    this->dir = direction;
    this->dir_string = dir_to_string(direction);
}

double DSRDS::get_x_offset(){
    return this->offsets.x;
}

double DSRDS::get_y_offset(){
    return this->offsets.y;
}

ez::pose DSRDS::get_offsets(){
    return this->offsets;
}

Dir DSRDS::get_dir(){
    return this->dir;
}

std::string DSRDS::get_dir_string(){
    return this->dir_string;
}

void DSRDS::measure_offsets(double read45, double read30, double read0){
    switch(dir){
        case Left:
        offsets.x = -Ya * read45 + Yb * read30 - Yc * read0;
        offsets.y = Xa * read45 - Xb * read30 + Xc * read0;
        break;
        case Right:
        offsets.x = Ya * read45 - Yb * read30 + Yc * read0;
        offsets.y = -Xa * read45 + Xb * read30 - Xc * read0;
        break;
        case Front:
        offsets.x = Xa * read45 - Xb * read30 + Xc * read0;
        offsets.y = Ya * read45 - Yb * read30 + Yc * read0;
        break;
        case Back:
        offsets.x = -Xa * read45 + Xb * read30 - Xc * read0;
        offsets.y = -Ya * read45 + Yb * read30 - Yc * read0;
        break;
    }
}