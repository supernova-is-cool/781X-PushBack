#include "dimensions.h"
#include "config.h"
#include "lemlib/chassis/trackingWheel.hpp"

robotConfig::Dimensions robotConfig::Dimensions::dimensions = {
    .trackWidth = dimensions::robot::TRACK_WIDTH,
    .driveWheelDiameter = lemlib::Omniwheel::NEW_325,
    .driveWheelRpm = 450,
    .vertEncDiameter = lemlib::Omniwheel::NEW_2,

    .vertEncDistance = -0.55f,
    .vertEncGearRatio = 1.f,
    .horiEncDiameter = lemlib::Omniwheel::NEW_2,

    .horiEncDistance = 0.5f,
    .horiEncGearRatio = 1.f,
    .drivetrainWidth = dimensions::robot::DRIVE_WIDTH,
    .drivetrainLength = dimensions::robot::DRIVE_LENGTH,
};