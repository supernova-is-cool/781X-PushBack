#include "config.h"

robotConfig::Sensors robotConfig::Sensors::sensors{
    .tubeExit{3},
    .vert{-16},
    .hori{-15},
    .imu{18},
    // TODO: Attach
    .front{3, {0, 0, 0}},
    .back{15, {-4.5, -2.5, M_PI}},
    .left{17, {-3, 5.5, M_PI_2}},
    .right{4, {-3, -5.5, -M_PI_2}},
};