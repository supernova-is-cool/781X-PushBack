#include "config.h"
#include "pros/adi.hpp"
#include <utility>

robotConfig::Pneumatics robotConfig::Pneumatics::pneumatics{
    .matchLoader = pros::adi::Pneumatics({10, 'C'}, false),
    .descore = pros::adi::Pneumatics{{10, 'D'}, false},
    .gate = pros::adi::Pneumatics{{10, 'A'}, false, true},
    .park = pros::adi::Pneumatics{'D', false},
    // TOOD: Configure intakeLift
    .intakeLift = pros::adi::Pneumatics{'B', false},
    .lift = PneumaticGroup{
        {pros::adi::Port{'C'},
         pros::adi::Port{pros::adi::ext_adi_port_pair_t(10, 'B')}},
        false}};