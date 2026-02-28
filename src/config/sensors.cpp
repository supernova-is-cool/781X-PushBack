#include "config.h"
#include "pros/adi.hpp"

robotConfig::Sensors robotConfig::Sensors::sensors{
    .intakeDistance{1},
    .vert{-14},
    .hori{-16},
    .imu{21},
    .front{3},
    .back{15},
    .left{17},
    .right{4},
};