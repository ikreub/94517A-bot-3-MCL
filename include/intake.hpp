#pragma once
#include "../include/subsystems.hpp"

namespace intake{
    void stop();
    void stop(int motor);
    void move(Rtype type, double speed);
    void opcontrol();
    inline bool intake_middle_speed = false;
    inline double speed;
    inline Rtype type;
    inline bool toggle = false;
    void wait_until_color(bool color, int timeout);
    bool get_color();
}