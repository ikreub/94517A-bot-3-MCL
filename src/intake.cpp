#include "../include/intake.hpp"
#include "pros/misc.h"

void intake::stop(){
    intake_1.move(0);
    intake_2.move(0);
}


void intake::stop(int motor){
    switch(motor){
        case 1:
        intake_1.move(0);
        break;
        case 2:
        intake_2.move(0);
        break;
    }
}

void intake::move(Rtype type, double speed){
    switch(type){
        case Rtype::Full:
        intake_1.move(speed);
        intake_2.move(speed);
        break;
        case Rtype::P_1:
        intake_1.move(speed);
        break;
        case Rtype::P_2:
        intake_2.move(speed);
        break;
    }
}

void intake::opcontrol(){
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        if(toggle){
            move(Rtype::Full, 127);
        }else{
            move(Rtype::P_1, 127);
        }
    }else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        move(Rtype::Full, -127);
    }else{
        stop();
    }
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
        toggle = !toggle;
    }
}