#include "robot.h"
const robotConfig robotConfig::config {
    .motors = robotConfig::Motors::motors,
    .pneumatics = robotConfig::Pneumatics::pneumatics,
    .sensors = robotConfig::Sensors::sensors,
    .dimensions = robotConfig::Dimensions::dimensions,
    .tunables = robotConfig::Tunables::tunables,
};