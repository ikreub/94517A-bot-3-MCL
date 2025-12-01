#include "../include/MCL/MCL.hpp"
#include "EZ-Template/util.hpp"

MCL::MCL(){
    for(double i = -Density; i <= Density; i++){
        for(double j = -Density; j <=Density; j++){
            if(sqrt(pow(i / Density,2) + pow(i / Density, 2)) <= 1){
                partical_spread.push_back({i / Density,j / Density});
            }
        }
    }
}

std::vector<pose> MCL::get_particals(double scaler, double theta){
    std::vector<pose> Output;
    for(unsigned int i = 0; i < partical_spread.size(); i++){
        Output.push_back({scaler * partical_spread[i].x, scaler * partical_spread[i].y, theta});
    }
    return Output;
}

std::vector<double> MCL::get_distances(){
    std::vector<double> Outputs;
    for(unsigned int i = 0; i  < sensors.size(); i++){
        Outputs.push_back(sensors[i].read());
    }
    return Outputs;
}

void MCL::MCL_task(){
    Certainty = 10;
    pose start_pose = chassis.odom_pose_get();
    pose d_pose;
    pose end_pose;
    pros::delay(100);
    std::vector<pose> p_gen = get_particals(Certainty, start_pose.theta);
    while(true){
        end_pose = chassis.odom_pose_get();
        d_pose = {end_pose.x - start_pose.x, end_pose.y - start_pose.y, end_pose.theta - start_pose.theta};
        for(int i = 0; i < p_gen.size(); i++){
            partical p(p_gen[i].x, p_gen[i].y, p_gen[i].theta, sensors);
            p.displace(d_pose.theta, d_pose.x, d_pose.y);
            p.get_weight(get_distances(), 15);
            particals.emplace_back(p);
        }

    }
}