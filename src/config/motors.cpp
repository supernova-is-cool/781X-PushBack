#include "config.h"

robotConfig::Motors robotConfig::Motors::motors{
    .left{11, -12, -14},
    .right{17, 19, -20},
    .intake{-1},
    .lever{{-21}, pros::v5::MotorGears::red},
};