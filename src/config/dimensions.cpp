#include "dimensions.h"
#include "config.h"
#include "lemlib/chassis/trackingWheel.hpp"

robotConfig::Dimensions robotConfig::Dimensions::dimensions = {
    .trackWidth = dimensions::robot::TRACK_WIDTH,
    .driveWheelDiameter = lemlib::Omniwheel::NEW_325,
    .driveWheelRpm = 450,
    .vertEncDiameter = 2 * 71.5 / 72.5,

    .vertEncDistance = 0.5f,
    .vertEncGearRatio = 1.f,
    .horiEncDiameter = 2 * 71.5 / 72.5,

    .horiEncDistance = 5. / 8,
    .horiEncGearRatio = 1.f,
    .drivetrainWidth = dimensions::robot::DRIVE_WIDTH,
    .drivetrainLength = dimensions::robot::DRIVE_LENGTH,
};