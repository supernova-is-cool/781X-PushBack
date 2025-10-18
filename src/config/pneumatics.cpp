#include "config.h"

robotConfig::Pneumatics robotConfig::Pneumatics::pneumatics{
    .filter = pros::adi::Pneumatics({6, 'C'}, true),
    .littleWill = pros::adi::Pneumatics({6, 'E'}, false),
    .leftElevator = pros::adi::Pneumatics({6, 'A'}, false),
    .rightElevator = pros::adi::Pneumatics{'F', false},
};