#include "config.h"

robotConfig::Pneumatics robotConfig::Pneumatics::pneumatics{
    .filter = pros::adi::Pneumatics{'E', true, true},
    .littleWill = pros::adi::Pneumatics{'H', false},
    .leftElevator = pros::adi::Pneumatics{'F', false},
    .rightElevator = pros::adi::Pneumatics{'A', false},
};