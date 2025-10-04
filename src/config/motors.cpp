#include "pros/motors.h"
#include "config.h"

robotConfig::Motors robotConfig::Motors::motors {
    .left {12, -13, -14},
    .right {18, -19, 20},
    .topStage {16, pros::v5::MotorGears::blue},
    .bottomStage {10, pros::v5::MotorGears::blue}
};