#include "../include/MCL/MCL-sensor.hpp"


MCLDS::MCLDS(Dir direction, int port, double offset_lr, double offset_fb) : dist_sen(port){
    offset_x = offset_lr;
    offset_y = offset_fb;
    dir = direction;
}

void MCLDS::set_offset_x(double offset){
    offset_x = offset;
}

void MCLDS::set_offset_y(double offset){
    offset_y = offset;
}  

void MCLDS::set_offset_xy(double offset_x, double offset_y){
    offset_x = offset_x;
    offset_y = offset_y;
}

void MCLDS::set_dir(Dir direction){
    dir = direction;
}

double MCLDS::get_offset_x(){
    return offset_x;
}

double MCLDS::get_offset_y(){
    return offset_y;
}

std::vector<double> MCLDS::get_offset_xy(){
    vector<double> offsets;
    offsets.push_back(offset_x);
    offsets.push_back(offset_y);
    return offsets;
}

Dir MCLDS::get_dir(){
    return dir;
}