#include "../include/MCL/MCL-particals.hpp"
#include "EZ-Template/util.hpp"

partical::partical(double x, double y, double theta, std::vector<MCLDS>& sensores){
    for(unsigned int i = 0; i < sensores.size(); i++){
        sensors.emplace_back(sensores[i]);
    }
    partical::x = x;
    partical::y = y;
    partical::theta = theta;
    weight = 0;
}

void partical::displace(double dtheta, double dx, double dy){
    theta = theta + dtheta;
    y = y + dy;
    x = x + dx;
}

std::vector<double> partical::get_sim_distance(){
    std::vector<double> returnlist;
    double returnval;
    double theta1;
    double theta2;
    double theta3;
    double theta4;
    double ttheta;
    pose sensorp; 
    // go through each sensor
    for(unsigned int i = 0; i < sensors.size(); i++){
        //define sensor pose
        switch(sensors[i].get_dir()){
            case Dir::Left:
            sensorp.theta = fmod(theta + M_PI_2, M_2_PI) - M_PI;
            break;
            case Dir::Right:
            sensorp.theta = fmod(theta + M_PI_2 + M_PI, M_2_PI) - M_PI;
            break;
            case Dir::Front:
            sensorp.theta = fmod(theta + M_PI, M_2_PI) - M_PI;
            break;
            case Dir::Back:
            sensorp.theta = fmod(theta, M_2_PI) - M_PI;
            break;
        }
        sensorp.x = x + sensors[i].get_offset_y() * cos(theta) - sensors[i].get_offset_y() * sin(theta);
        sensorp.y = y - sensors[i].get_offset_y() * sin(theta) - sensors[i].get_offset_x() * cos(theta); 
        // define angles to figure out which wall its looking at
        theta1 = -atan((144 - (sensorp.y)) / (144 - (sensorp.x))) + M_PI_2;
        theta2 = atan((144 - sensorp.y) / sensorp.x) - M_PI_2;
        theta3 = -atan(sensorp.y / sensorp.x) - M_PI_2;
        theta4 = atan(sensorp.y / (144 - sensorp.x)) + M_PI_2;
        // actual distance calculation
        if((theta1 <= sensorp.theta) && (sensorp.theta < theta4)){
            returnval = 25.4 * sqrt(pow(tan(sensorp.theta - M_PI_2), 2)) * abs(sensorp.x - 144);
        }else if((theta2 <= sensorp.theta) && (sensorp.theta < theta1)){
            returnval = 25.4 * sqrt(1 + (1 / pow(tan(sensorp.theta - M_PI_2), 2))) * abs(sensorp.y - 144);
        }else if((theta3 <= sensorp.theta) && (sensorp.theta < theta2)){
            returnval = 25.4 * sqrt(1 + pow(tan(sensorp.theta - M_PI_2), 2)) * abs(sensorp.x);
        }else{
            returnval = 25.4 * sqrt(1 + 1 / pow(tan(sensorp.theta - M_PI_2), 2)) * abs(sensorp.y);
        }
        returnlist.push_back(returnval);
    }
    return returnlist;
}

void partical::get_weight(std::vector<double> sensor_vals, double allowed_deviation){
    std::vector<double> weighted;
    std::vector<double> sim_distances = get_sim_distance();
    double W;
    int count;
    for(unsigned int i; i < sensor_vals.size(); i++){
        W = abs(sensor_vals[i] - sim_distances[i]) / 25.4;
        if(W <= allowed_deviation){
            weighted.push_back(W);
            count++;
            weight = weight + W;
        }
        if(i == sensor_vals.size() - 1){
            weight = weight / count;
        }
    }
}