#include "config.h"
#include "pros/adi.hpp"

robotConfig::Sensors robotConfig::Sensors::sensors{
    .intakeFilter{6},
    .vert{30},
    .hori{-13},
    .imu{12},
};