#include "dimensions.h"
#include "config.h"

robotConfig::Dimensions robotConfig::Dimensions::dimensions = {
    .trackWidth = dimensions::robot::TRACK_WIDTH,
    .driveWheelDiameter = 2.75f * 84.0 / 75.5 * 84.0 /
                          83.0 /** Scale factor to make more accurate */,
    .driveWheelRpm = 600,
    .vertEncDiameter = lemlib::Omniwheel::NEW_2,

    .vertEncDistance = 0.4f,
    .vertEncGearRatio = 1.f,
    .horiEncDiameter = lemlib::Omniwheel::NEW_2,

    .horiEncDistance = -4.18f,
    .horiEncGearRatio = 1.f,
    .drivetrainWidth = dimensions::robot::DRIVE_WIDTH,
    .drivetrainLength = dimensions::robot::DRIVE_LENGTH,
};