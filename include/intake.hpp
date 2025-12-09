#pragma once
#include "../include/subsystems.hpp"

namespace intake{
    void stop();
    void stop(int motor);
    void move(Rtype type, double speed);
    void opcontrol();
    inline bool toggle = true;
}