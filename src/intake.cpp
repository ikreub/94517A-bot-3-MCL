#include "../include/intake.hpp"
#include "pros/misc.h"
#include "subsystems.hpp"
#include "antijam.hpp"

#define blue_min 220
#define blue_max 230
#define red_min 350
#define red_max 10

void intake::stop(){
    intake::speed = 0;
    if(!antiJam::disable){
    intake_1.move(0);
    intake_2.move(0);
    }
}


void intake::stop(int motor){
    if(!antiJam::disable){
    intake::speed = 0;
    switch(motor){
        case 1:
        intake_1.move(0);
        break;
        case 2:
        intake_2.move(0);
        break;
    }
    }
}

void intake::move(Rtype type, double speed){
    intake::speed = speed;
    if(!antiJam::disable){
    switch(type){
        case Full:
        intake_1.move(speed);
        intake_2.move(speed);
        break;
        case P_1:
        intake_1.move(speed);
        break;
        case P_2:
        intake_2.move(speed);
        break;
    }
    }
}

void intake::opcontrol(){
    speed = (IntakeRaise.get() == true) ? (intake_middle_speed) ? 65 : 127 : 127;
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        if(toggle){
            move(Rtype::Full, speed);
        }else{
            move(Rtype::P_1, speed);
        }
    }else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        move(Rtype::Full, -127);
    }else{
        stop();
    }
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
        toggle = !toggle;
    }
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
        intake_middle_speed = !intake_middle_speed;
    }
}

void intake::wait_until_color(bool color, int timeout){
    bool is_true = false;
    int time = 0;
    int hue;
    while(!is_true){
        hue = colorStop.get_hue();
        if(colorStop.get_proximity() == 255){
            if(color){
                if(red_min < hue || red_max > hue ){
                    is_true = true;
                }
            }else{
                if(blue_min < hue && hue < blue_max){
                    is_true = true;
                }
            }
        }
        if(time >= timeout){
            is_true =  true;
        }
        time += 10;
        pros::delay(10);
    }
    pros::delay(50);
}

bool intake::get_color(){
    int hue = colorStop.get_hue();
    if(blue_min < hue && hue < blue_max){
        return false;
    }else{
        return true;
    }
}