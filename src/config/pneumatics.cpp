#include "config.h"

robotConfig::Pneumatics robotConfig::Pneumatics::pneumatics{
    .filter = pros::adi::Pneumatics{'E', true, true},
    .littleWill = pros::adi::Pneumatics{'H', false},
    .elevator = pros::adi::Pneumatics{'F', false},
};