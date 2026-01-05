#pragma once

#include "EZ-Template/api.hpp"
#include "EZ-Template/piston.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor intake_1(4);
inline pros::Motor intake_2(-5);
// inline pros::adi::DigitalIn limit_switch('A');

inline ez::Piston MatchLoad(!'G');
inline ez::Piston IntakeRaise('H');
inline ez::Piston DoublePark(!'F');

enum class Rtype{
    Full,
    P_1,
    P_2,
};
