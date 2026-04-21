#include "config.h"
#include "dimensions.h"

robotConfig::Sensors robotConfig::Sensors::sensors{
    .tubeExit{3},
    .vert{-16},
    .hori{-15},
    .imu{18},
    // TODO: Attach
    .front{8, {0, 0, 0}},
    .back{7, {-4.5, -2.5, M_PI}},
    .left{5, {-3.75, dimensions::robot::DRIVE_WIDTH / 2, M_PI_2}},
    .right{9, {-3.75, -dimensions::robot::DRIVE_WIDTH / 2, -M_PI_2}},
    .matchloader{4}};