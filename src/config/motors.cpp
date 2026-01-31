#include "pros/motors.h"
#include "config.h"

robotConfig::Motors robotConfig::Motors::motors {
    .left {-18, -19, -20},
    .right {11, 12, 13},
    .topStage {-1, pros::v5::MotorGears::blue},
    .bottomStage {10, pros::v5::MotorGears::blue}
};