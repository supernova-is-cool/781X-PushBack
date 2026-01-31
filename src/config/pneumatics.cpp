#include "config.h"
#include "pros/adi.hpp"

robotConfig::Pneumatics robotConfig::Pneumatics::pneumatics{
    .matchLoader = pros::adi::Pneumatics('H', false),
    .descore = pros::adi::Pneumatics{'G', false},
    .bottom_gate = pros::adi::Pneumatics{'A', false},
    .top_gate = pros::adi::Pneumatics{'F', false},
    .park = pros::adi::Pneumatics{'D', false},
    .clamp = pros::adi::Pneumatics{'C', false}
};