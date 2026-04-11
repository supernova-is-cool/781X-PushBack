#include "config.h"

robotConfig::Pneumatics robotConfig::Pneumatics::pneumatics{
    .matchLoader = pros::adi::Pneumatics('H', false),
    .descore = pros::adi::Pneumatics{'G', false},
    .gate = pros::adi::Pneumatics{'A', false},
    .park = pros::adi::Pneumatics{'D', false},
    .lift = PneumaticGroup{{'B', 'E'}, false, true}};