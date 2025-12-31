#include "config.h"
#include "pros/adi.hpp"

robotConfig::Pneumatics robotConfig::Pneumatics::pneumatics{
    .matchLoader = pros::adi::Pneumatics('G', false),
    .lift = pros::adi::Pneumatics('H', false),
    .descore = pros::adi::Pneumatics{'F', false},
    .odomLift = pros::adi::Pneumatics{'E', true, true}
};