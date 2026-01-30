#pragma once

#include "EZ-Template/api.hpp"
#include "EZ-Template/piston.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor intake_1(4);
inline pros::Motor intake_2(-6);
// inline pros::adi::DigitalIn limit_switch('A');

inline ez::Piston MatchLoad('F');
inline ez::Piston IntakeRaise('H', true);
inline ez::Piston DoublePark('G', true);
inline ez::Piston Wing('E', true);

inline pros::Optical colorStop(14);

enum Rtype{
    Full,
    P_1,
    P_2,
};
