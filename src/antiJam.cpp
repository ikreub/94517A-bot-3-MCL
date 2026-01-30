#include "antijam.hpp"
#include "EZ-Template/util.hpp"
#include "subsystems.hpp"
#include "intake.hpp"

namespace antiJam{
    void Task(){
        
        while(1){
            if(!disabled){
                Last_speed = intake::speed;
                if(abs(Last_speed) > 0){
                    pros::delay(100);
                    if((intake_1.get_efficiency() == 0)){
                        disable = true;
                        intake_1.move(-127);
                        pros::delay(150);
                        intake_1.move(Last_speed);
                        disable = false;
                    }
                }
            }
            pros::delay(ez::util::DELAY_TIME);
        }
    }

}